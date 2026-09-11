
/**
 * @brief Formation offboard control - staged testing version
 * @file formation_offboard_control.cpp
 * @addtogroup examples
 *
 * 支援：
 *  1. 統一 start 觸發（訂閱 /formation/start，收到前完全不發送任何指令）
 *  2. 原地起飛懸停（用「收到 start 當下的實際位置」當 home，而不是寫死座標）
 *  3. Offboard 只強制介入一次 —— 一旦成功進入 Offboard+Armed，就不再搶控制權，
 *     讓遙控器可以之後安全接管 leader
 *  4. follow_leader 為 runtime 參數，可以用 `ros2 param set` 動態切換
 *     懸停測試模式 / 編隊模式，不需要重啟 node
 */

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**
 * 碰撞避免用：記錄「其他機」（除了 leader 以外，例如另一台 follower）的即時位置。
 * 用 shared_ptr 存放，讓訂閱的 lambda 可以直接捕捉並更新這個物件。
 */
/**
 * 位置合理性檢查用的狀態——比較「這一筆」跟「上一筆原始值」之間隱含的速度，
 * 超過合理上限就判定不可信。設計細節見 OffboardControl::check_position_plausible()。
 * 放在檔案外層，這樣 WatchedVehicle 才能直接用到同一個型別。
 */
struct PositionSanityState
{
	bool has_last_raw{false};
	double last_raw_x{0.0}, last_raw_y{0.0}, last_raw_z{0.0};
	rclcpp::Time last_raw_time;
	int reject_streak{0};
	int recovery_streak{0};      // 已宣告不可信之後，連續幾筆合理樣本了
	bool declared_invalid{false};   // 目前是不是處於「已經宣告不可信」的狀態
};

struct WatchedVehicle
{
	int mav_id{0};
	double x{0.0}, y{0.0}, z{0.0};
	bool valid{false};
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr sub;
	PositionSanityState sanity;   // 每台被監控的機各自獨立一份合理性檢查狀態
};

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("formation_offboard_control")
	{
		declare_and_read_parameters();

		// 參數驗證 callback：在 formation_presets_ 已經載入完成之後才註冊，避免攔截到
		// declare_and_read_parameters() 內部自己的初始宣告。攔截的是之後操作者透過
		// `ros2 param set` 做的變更——不安全的切換直接在這裡拒絕，操作者下指令的當下
		// 就會在自己的 terminal 看到拒絕原因，不需要另外去看 log 才知道發生什麼事。
		param_callback_handle_ = this->add_on_set_parameters_callback(
			[this](const std::vector<rclcpp::Parameter> &params) {
				return this->validate_parameter_updates(params);
			}
		);

		std::string ns = mav_ns_prefix();

		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>(ns + "/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>(ns + "/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>(ns + "/fmu/in/vehicle_command", 10);

		rclcpp::QoS px4_qos(10);
		px4_qos.best_effort();

		// 訂閱「自己」的即時位置，用來在 start 當下捕捉 home 座標（原地起飛用）
		// topic 後綴改成可設定參數 (position_topic_suffix_)：不同 PX4/px4_msgs 版本，
		// vehicle_local_position 是否帶 "_v1" 版本後綴可能不一樣，上機發現對不上時
		// 改 yaml 就好，不用重編。C++ 型別本身不帶版本號，還是 VehicleLocalPosition，
		// 不要因為 topic 名稱有沒有 _v1 就誤用 VehicleLocalPositionV1（那是舊版、已被取代的定義）。
		own_position_sub_ = this->create_subscription<VehicleLocalPosition>(
			ns + position_topic_suffix_,
			px4_qos,
			[this](const VehicleLocalPosition::SharedPtr msg) {
				bool became_invalid = false, recovered = false, confirming_recovery = false;
				bool plausible = check_position_plausible(
					own_pos_sanity_, msg->x, msg->y, msg->z, became_invalid, recovered, confirming_recovery);

				if (plausible) {
					current_x_ = msg->x;
					current_y_ = msg->y;
					current_z_ = msg->z;
					current_position_valid_ = true;
					if (recovered) {
						RCLCPP_WARN(this->get_logger(),
							"[MAV%d] Own position plausibility RECOVERED - resuming normal operation.",
							mav_id_);
					}
				} else if (confirming_recovery) {
					RCLCPP_WARN_THROTTLE(
						this->get_logger(), *this->get_clock(), 500,
						"[MAV%d] Own position looks OK again but still confirming stability "
						"(%d/%d consecutive good samples) - still holding last known good position.",
						mav_id_, own_pos_sanity_.recovery_streak, position_recovery_streak_required_
					);
				} else {
					RCLCPP_WARN_THROTTLE(
						this->get_logger(), *this->get_clock(), 500,
						"[MAV%d] ** IMPLAUSIBLE OWN POSITION ** rejecting sample (%.2f, %.2f, %.2f) "
						"- implied speed too high, holding last known good position.",
						mav_id_, msg->x, msg->y, msg->z
					);
					if (became_invalid) {
						current_position_valid_ = false;
						RCLCPP_ERROR(this->get_logger(),
							"[MAV%d] ** OWN POSITION ESTIMATE UNRELIABLE ** (%d consecutive implausible "
							"samples) - freezing setpoint at last known position. Check EKF/mocap; "
							"consider manual takeover or emergency land.",
							mav_id_, position_reject_streak_limit_
						);
					}
				}
			}
		);

		// 訂閱「自己」的飛控狀態，用來判斷是否已經成功進入 Offboard+Armed，
		// 一旦成功就不再強制搶控制權（讓遙控器之後能安全接管）。
		// topic 後綴也改成可設定參數 (vehicle_status_topic_suffix_)：vehicle_status 跟
		// vehicle_local_position 是不是同樣有 "_v1" 後綴，兩者可能不一致（各自訊息版本
		// 獨立編號），這是實機上很容易對不上、卻完全沒有錯誤訊息提示的地雷——
		// 對不上時這個訂閱會安靜地永遠收不到資料，nav_state_/arming_state_ 會卡在
		// 建構時的預設值 0，導致 node 誤判「一直沒成功進 Offboard」而不斷重試 arm，
		// 但飛機可能其實已經真的起飛了。懷疑卡住時，先用
		//   ros2 topic list | grep vehicle_status
		// 確認真正的 topic 名稱，對不上就改這個參數，不用重編。
		vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
			ns + vehicle_status_topic_suffix_,
			px4_qos,
			[this](const VehicleStatus::SharedPtr msg) {
				nav_state_ = msg->nav_state;
				arming_state_ = msg->arming_state;
			}
		);

		// 備援判斷依據：vehicle_command_ack。
		// 現場實測發現某些飛控韌體設定下 vehicle_status 沒有被 uXRCE-DDS 橋接出來
		// （topic 有註冊但永遠收不到資料，nav_state_/arming_state_ 卡在預設值 0，
		// 導致 node 誤判「一直沒成功進 Offboard」而不斷重試 arm，即使飛控其實已經真的
		// 成功了）。vehicle_command_ack 是 PX4 對我們送出的 VehicleCommand 的直接回覆，
		// 不依賴 vehicle_status，用它來確認 DO_SET_MODE / ARM_DISARM 有沒有被接受，
		// 當作 offboard_engaged_once_ 判斷的備援依據。
		// result==0 對應 MAV_RESULT_ACCEPTED。
		vehicle_command_ack_sub_ = this->create_subscription<VehicleCommandAck>(
			ns + vehicle_command_ack_topic_suffix_,
			px4_qos,
			[this](const VehicleCommandAck::SharedPtr msg) {
				if (msg->result != 0) {
					return;   // 只關心被接受的 ack，被拒絕的不採用
				}
				if (msg->command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE) {
					mode_ack_accepted_ = true;
				} else if (msg->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) {
					arm_ack_accepted_ = true;
				}
			}
		);

		// Follower 才需要訂閱 leader 的即時位置
		if (!is_leader_) {
			std::string leader_ns = "/MAV" + std::to_string(leader_mav_id_);

			leader_position_sub_ = this->create_subscription<VehicleLocalPosition>(
				leader_ns + position_topic_suffix_,
				px4_qos,
				[this](const VehicleLocalPosition::SharedPtr msg) {
					bool became_invalid = false, recovered = false, confirming_recovery = false;
					bool plausible = check_position_plausible(
						leader_pos_sanity_, msg->x, msg->y, msg->z,
						became_invalid, recovered, confirming_recovery);

					if (plausible) {
						leader_x_ = msg->x;
						leader_y_ = msg->y;
						leader_z_ = msg->z;
						leader_vx_ = msg->vx;
						leader_vy_ = msg->vy;
						if (yaw_source_ != "mocap") {
							// yaw_source_=="mocap" 時，heading 改由 leader_mocap_odom_sub_ 的
							// callback 填值，這裡不要覆寫，避免兩個來源互相打架。
							leader_heading_ = msg->heading;
							leader_heading_good_for_control_ = msg->heading_good_for_control;
						}
						leader_position_valid_ = true;
						last_leader_msg_time_ = this->get_clock()->now();
						if (recovered) {
							RCLCPP_WARN(this->get_logger(),
								"[MAV%d] Leader(MAV%d) position plausibility RECOVERED - resuming tracking.",
								mav_id_, leader_mav_id_);
						}
					} else if (confirming_recovery) {
						RCLCPP_WARN_THROTTLE(
							this->get_logger(), *this->get_clock(), 500,
							"[MAV%d] Leader(MAV%d) position looks OK again but still confirming stability "
							"(%d/%d consecutive good samples) - still holding last known good leader position.",
							mav_id_, leader_mav_id_, leader_pos_sanity_.recovery_streak,
							position_recovery_streak_required_
						);
					} else {
						RCLCPP_WARN_THROTTLE(
							this->get_logger(), *this->get_clock(), 500,
							"[MAV%d] ** IMPLAUSIBLE LEADER(MAV%d) POSITION ** rejecting sample "
							"(%.2f, %.2f, %.2f) - holding last known good leader position.",
							mav_id_, leader_mav_id_, msg->x, msg->y, msg->z
						);
						if (became_invalid) {
							leader_position_valid_ = false;
							RCLCPP_ERROR(this->get_logger(),
								"[MAV%d] ** LEADER(MAV%d) POSITION ESTIMATE UNRELIABLE ** (%d consecutive "
								"implausible samples) - freezing formation target at last known position, "
								"NOT chasing this leader position until it recovers.",
								mav_id_, leader_mav_id_, position_reject_streak_limit_
							);
						}
					}
				}
			);

			// yaw_source_=="mocap" 時，額外訂閱 mocap_px4_bridge.cpp 發布給 PX4 的
			// vehicle_visual_odometry，直接從裡面萃取 leader yaw，完全繞開 PX4 EKF2 的
			// yaw_align/heading_good_for_control gating（這幾輪 session 查出來的一連串
			// 內部門檻：磁力計要飛到 >1.5m 才會重新對齊、POSE_FRAME_FRD 會讓 yaw_align
			// 永遠鎖 false...）。這個 topic 是「餵給 EKF2 的原始輸入」，不是 EKF2 融合後的
			// 輸出，訂閱它不受那些門檻影響。SITL 沒有 mocap_px4_bridge，這個分支只在真機
			// 搭配 yaw_source: mocap 時才會建立訂閱。
			if (yaw_source_ == "mocap") {
				leader_mocap_odom_sub_ = this->create_subscription<VehicleOdometry>(
					leader_ns + mocap_odometry_topic_suffix_,
					px4_qos,
					[this](const VehicleOdometry::SharedPtr msg) {
						leader_heading_ = yaw_from_quat(msg->q);
						leader_heading_good_for_control_ = true;
						last_leader_mocap_msg_time_ = this->get_clock()->now();
					}
				);
			}
		}

		// Leader 才需要訂閱搖桿：把搖桿偏移量當成「期望速度」的輸入源，
		// 不再靠遙控器切模式離開 Offboard 來搶控制權——leader 全程留在 Offboard，
		// 搖桿只是持續改變我們發送的 setpoint。
		// 注意：topic 後綴 (rc_topic_suffix_) 請務必上機用
		//   ros2 topic echo <ns>/fmu/out/manual_control_setpoint
		// 實測搖桿有沒有反應在 pitch/roll 欄位上再確認，不同 PX4 版本欄位名稱可能略有差異。
		if (is_leader_ && teleop_enabled_) {
			manual_control_sub_ = this->create_subscription<ManualControlSetpoint>(
				ns + rc_topic_suffix_,
				px4_qos,
				[this](const ManualControlSetpoint::SharedPtr msg) {
					rc_pitch_ = msg->pitch;
					rc_roll_ = msg->roll;
					rc_yaw_ = msg->yaw;
					rc_throttle_ = msg->throttle;
					rc_valid_ = msg->valid;
					last_rc_msg_time_ = this->get_clock()->now();
					rc_data_received_ = true;
				}
			);
		}

		// 碰撞避免：訂閱 collision_watch_mav_ids_ 列出的「其他機」位置（例如另一台 follower）。
		// leader 不會被重複訂閱兩次：follower 對 leader 的位置已經有專門的 leader_position_sub_，
		// 這裡自動略過跟自己、跟 leader 重複的 id。
		for (int64_t watch_id : collision_watch_mav_ids_) {
			if (watch_id == mav_id_) continue;
			if (!is_leader_ && watch_id == leader_mav_id_) continue;

			auto wv = std::make_shared<WatchedVehicle>();
			wv->mav_id = static_cast<int>(watch_id);
			std::string watch_ns = "/MAV" + std::to_string(watch_id);

			wv->sub = this->create_subscription<VehicleLocalPosition>(
				watch_ns + position_topic_suffix_,
				px4_qos,
				[this, wv](const VehicleLocalPosition::SharedPtr msg) {
					bool became_invalid = false, recovered = false, confirming_recovery = false;
					bool plausible = check_position_plausible(
						wv->sanity, msg->x, msg->y, msg->z, became_invalid, recovered, confirming_recovery);

					if (plausible) {
						wv->x = msg->x;
						wv->y = msg->y;
						wv->z = msg->z;
						wv->valid = true;
						if (recovered) {
							RCLCPP_WARN(this->get_logger(),
								"[MAV%d] Watched vehicle MAV%d position plausibility RECOVERED.",
								mav_id_, wv->mav_id);
						}
					} else if (confirming_recovery) {
						RCLCPP_WARN_THROTTLE(
							this->get_logger(), *this->get_clock(), 500,
							"[MAV%d] Watched vehicle MAV%d position looks OK again but still confirming "
							"stability (%d/%d consecutive good samples).",
							mav_id_, wv->mav_id, wv->sanity.recovery_streak, position_recovery_streak_required_
						);
					} else {
						RCLCPP_WARN_THROTTLE(
							this->get_logger(), *this->get_clock(), 500,
							"[MAV%d] ** IMPLAUSIBLE MAV%d POSITION ** (watched for collision avoidance) "
							"rejecting sample (%.2f, %.2f, %.2f) - holding last known position.",
							mav_id_, wv->mav_id, msg->x, msg->y, msg->z
						);
						if (became_invalid) {
							wv->valid = false;
							RCLCPP_ERROR(this->get_logger(),
								"[MAV%d] ** MAV%d POSITION ESTIMATE UNRELIABLE ** (watched for collision "
								"avoidance) - ignoring this vehicle for avoid_collisions until it recovers.",
								mav_id_, wv->mav_id
							);
						}
					}
				}
			);

			watched_vehicles_.push_back(wv);
		}

		// 統一 start 觸發：收到 /formation/start (data=true) 之前，完全不送任何指令。
		// 所有機的 node 可以先啟動待命，等你發一個 start 才「同時」開始起飛程序。
		rclcpp::QoS start_qos(10);
		start_qos.reliable();

		start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
			start_topic_,
			start_qos,
			[this](const std_msgs::msg::Bool::SharedPtr msg) {
				if (msg->data && !start_requested_) {
					start_requested_ = true;
					RCLCPP_INFO(this->get_logger(), "[MAV%d] Start signal received.", mav_id_);
				}
			}
		);

		// 統一 land 觸發：跟 start 用同一種「單一指令、全部一起動」的設計。
		// 收到 /formation/land (data=true) 後，不管目前是 teleop / 編隊 / 懸停哪種模式，
		// 一律凍結目前的 x/y、只讓 z 慢慢降回 home（起飛時的地面高度）。
		// 不會自動 disarm——降到地面後只會持續印 WARN 提醒操作者手動用遙控器/QGC 斷電確認。
		land_sub_ = this->create_subscription<std_msgs::msg::Bool>(
			"/formation/land",
			start_qos,
			[this](const std_msgs::msg::Bool::SharedPtr msg) {
				if (msg->data && !landing_requested_) {
					landing_requested_ = true;
					RCLCPP_WARN(this->get_logger(), "[MAV%d] Land signal received - descending to ground.", mav_id_);
				}
			}
		);

		// 起飛同步用的「集合點(barrier)」：home capture 完就持續回報 /formation/ready，
		// 全域共用同一個 topic（所有機都在同一個上面互相聽），不用 namespace 前綴。
		ready_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/formation/ready", start_qos);
		ready_sub_ = this->create_subscription<std_msgs::msg::Int32>(
			"/formation/ready",
			start_qos,
			[this](const std_msgs::msg::Int32::SharedPtr msg) {
				received_ready_ids_.insert(msg->data);
			}
		);

		// 第二層 barrier：arm 成功後持續回報 /formation/armed_ready，等大家都 armed 才一起爬升。
		armed_ready_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/formation/armed_ready", start_qos);
		armed_ready_sub_ = this->create_subscription<std_msgs::msg::Int32>(
			"/formation/armed_ready",
			start_qos,
			[this](const std_msgs::msg::Int32::SharedPtr msg) {
				received_armed_ready_ids_.insert(msg->data);
			}
		);

		offboard_setpoint_counter_ = 0;

		timer_ = this->create_wall_timer(
			100ms,
			[this]() { this->timer_callback(); }
		);

		RCLCPP_INFO(
			this->get_logger(),
			"Formation node ready: MAV%d role=%s (waiting for start signal on %s)",
			mav_id_, is_leader_ ? "LEADER" : "FOLLOWER", start_topic_.c_str()
		);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr leader_position_sub_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr leader_mocap_odom_sub_;   // yaw_source_=="mocap" 時才建立
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr own_position_sub_;
	rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr land_sub_;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ready_publisher_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ready_sub_;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr armed_ready_publisher_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr armed_ready_sub_;
	rclcpp::Subscription<ManualControlSetpoint>::SharedPtr manual_control_sub_;
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

	uint64_t offboard_setpoint_counter_;

	// ---- 基本參數 ----
	int mav_id_;
	bool is_leader_;
	int leader_mav_id_;
	double yaw_;
	double hover_altitude_;                    // 原地起飛要爬升的高度 (公尺，正值)
	std::string start_topic_;
	double leader_timeout_ms_;
	// 這幾個 topic 後綴改成可設定參數，上機發現對不上時改 yaml 就好，不用重編。
	std::string position_topic_suffix_{"/fmu/out/vehicle_local_position_v1"};
	std::string vehicle_status_topic_suffix_{"/fmu/out/vehicle_status_v1"};
	// 現場實測這個 topic 不需要 _v1 後綴，但一樣做成可設定參數，避免不同韌體版本差異。
	std::string vehicle_command_ack_topic_suffix_{"/fmu/out/vehicle_command_ack"};
	// leader heading 的來源："px4"（預設，讀 vehicle_local_position.heading，會受
	// EKF2 的 yaw_align/heading_good_for_control 那套內部門檻限制）或 "mocap"（直接
	// 讀 mocap_px4_bridge.cpp 發布給 PX4 的 vehicle_visual_odometry，完全繞開 EKF2
	// gating，只有真機 + mocap 場地能用，SITL 沒有這個 topic）。
	std::string yaw_source_{"px4"};
	std::string mocap_odometry_topic_suffix_{"/fmu/in/vehicle_visual_odometry"};
	double mocap_yaw_timeout_ms_{500.0};
	rclcpp::Time last_leader_mocap_msg_time_;

	// vehicle_status 失效時的備援依據：DO_SET_MODE / ARM_DISARM 有沒有各自被接受過一次。
	// 只要兩個都 true，就採信「已經成功進 Offboard+Armed」，不用等 vehicle_status 更新。
	bool mode_ack_accepted_{false};
	bool arm_ack_accepted_{false};

	// ---- 隊形 preset 與主動 offset ----
	struct FormationOffset { double x, y, z; };

	// PositionSanityState 定義在檔案外層（WatchedVehicle 之前），這裡不用再定義一次。
	std::map<std::string, FormationOffset> formation_presets_;
	std::string formation_mode_{"echelon"};
	bool auto_calibrate_offset_{true};

	// 目前實際套用的 offset（固定座標系，即 PX4 local NED frame，不隨飛機轉動）：
	// auto_calibrate 反推完成後用反推值，反推前或 auto_calibrate=false 時用 preset 數值
	double active_offset_x_{0.0}, active_offset_y_{0.0}, active_offset_z_{0.0};
	bool offset_calibrated_{false};   // auto_calibrate 反推是否已完成（固定座標系下的 offset）

	// ---- 剛體編隊：leader 機身座標系（forward/right）下的固定 offset ----
	// 跟上面 offset_calibrated_ 是「兩個獨立事件」：
	// offset_calibrated_ 只反推「固定座標系」下的相對位置差（不管 leader 朝哪）；
	// yaw_offset_locked_ 則是在「rotate_offset 第一次變成 true」的那一刻，
	// 用「當下」的 leader heading 把固定座標系下的相對位置差反推回 leader 機身座標系，
	// 之後才用這個機身座標系固定值配合 leader 當下 heading 轉回固定座標系算目標點。
	// 這樣不管是開機就直接開 yaw_follow_leader，還是像目前流程先做位置編隊、
	// 之後才切換開啟 yaw follow，兩種情況都能在「真正開始套用旋轉」那一刻，
	// 用當下最新的相對位置重新鎖定機身座標系 offset，避免用到不對應的旋轉基準。
	bool yaw_offset_locked_{false};
	double body_offset_x_{0.0}, body_offset_y_{0.0};   // leader 機身座標系(forward,right)下鎖定的固定 offset

	// ---- start / home 狀態 ----
	bool start_requested_{false};
	bool started_{false};

	// ---- 起飛同步「集合點(barrier)」----
	// 所有隊形成員(formation_mav_ids_)都回報 ready 後，再等 liftoff_grace_period_s_ 這段
	// 固定緩衝時間(各機各自本地倒數，不需要跨機時鐘同步)，才真正開始 arm+Offboard。
	std::vector<int64_t> formation_mav_ids_;
	double liftoff_grace_period_s_{1.0};
	std::set<int64_t> received_ready_ids_;
	bool quorum_reached_{false};
	rclcpp::Time quorum_time_;

	// ---- 第二層 barrier：等大家都真的 arm 成功才一起爬升 ----
	double liftoff_after_arm_timeout_s_{10.0};
	std::set<int64_t> received_armed_ready_ids_;
	bool waiting_for_armed_quorum_{false};
	bool liftoff_authorized_{false};
	rclcpp::Time armed_wait_start_time_;

	// ---- 降落（/formation/land）----
	// 不做自動 disarm——降到地面高度附近後只持續印 WARN 提醒，斷電永遠人工用遙控器/QGC 確認。
	bool landing_requested_{false};
	bool landing_target_captured_{false};
	double landing_fx_{0.0}, landing_fy_{0.0};   // 收到 land 那一刻凍結住的 x/y，只有 z 會繼續變化
	double landed_height_margin_m_{0.05};
	// 降落目標比 home_z_ 再往下(NED z 增加=更低)多推這麼多，避免飛機只是「追到記錄的地面
	// 高度」就懸在半空中不下去(mocap 零點雜訊 / position controller 收斂殘留誤差常見現象)。
	double land_extra_descend_m_{0.20};
	// leader 手動 throttle 降落用的影子狀態（is_leader_ && teleop_enabled_ 時才會用到，
	// 見 compute_landing_manual_descent_z()）。跟 teleop_x_/y_、follow_x_/y_ 是同一種
	// 「開迴路積分」設計，不直接拿 current_z_ 當累積起點，避免被控制器雜訊干擾這一步該給多少速度。
	double landing_manual_z_{0.0};
	bool has_last_landing_manual_time_{false};
	rclcpp::Time last_landing_manual_time_;
	// 接近預估地面高度(land_target_z = home_z_ + land_extra_descend_m_)時，即使 throttle
	// 桿壓到底，下降速度也會朝這個「爬行速度」收斂，而不是收斂到 0——如果收斂到 0，
	// 一旦預估地面高度不準(mocap 零點誤差)，會重新製造「卡在固定高度、飛機沒有真的落地」
	// 的老問題。收斂到一個很慢但非零的速度，才能在保護撞地的同時仍然讓飛機繼續往下探。
	double landing_crawl_speed_mps_{0.05};
	// 離預估地面高度還有多少公尺開始線性減速（見 landing_crawl_speed_mps_ 說明）。
	double landing_slowdown_radius_m_{0.3};

	double current_x_{0.0}, current_y_{0.0}, current_z_{0.0};
	bool current_position_valid_{false};

	double home_x_{0.0}, home_y_{0.0}, home_z_{0.0};
	bool home_captured_{false};

	// ---- leader 位置 (follower 用) ----
	double leader_x_{0.0}, leader_y_{0.0}, leader_z_{0.0};
	double leader_vx_{0.0}, leader_vy_{0.0};   // 外插補償延遲用
	double leader_heading_{0.0};   // NED yaw (rad)，用來把 body-frame offset 旋轉成世界座標
	bool leader_heading_good_for_control_{false};   // PX4 EKF2 自報 heading 是否夠可靠，不可靠時不套用旋轉
	bool leader_position_valid_{false};
	rclcpp::Time last_leader_msg_time_;

	// ---- 位置合理性檢查（own + leader 各一份獨立狀態）----
	// 事故教訓：EKF/mocap 一旦發散，回報的位置可能瞬間跳到物理上不可能的數值，
	// 這裡加一層獨立於角色的合理性過濾：連續兩筆之間隱含的速度
	// 超過 max_plausible_speed_mps_ 就拒絕採用，連續拒絕達到 position_reject_streak_limit_
	// 次才判定「這個位置來源目前不可信」，避免單一雜訊突波就誤判。
	PositionSanityState own_pos_sanity_;
	PositionSanityState leader_pos_sanity_;
	double max_plausible_speed_mps_{2.0};
	int position_reject_streak_limit_{3};
	int position_recovery_streak_required_{5};
	// 固定延遲常數猜測值（秒），現階段先設 0（不補償），等實測知道大概延遲量再調。
	// 實際套用的延遲 = 這個常數 + 「這筆 leader 資料距離現在已經放了多久沒更新」（用 follower
	// 自己的時鐘算，不需要跨機時鐘同步），兩者相加乘上 leader 當下速度做外插。
	double leader_link_latency_estimate_s_{0.0};

	// ---- offboard 介入狀態 ----
	bool offboard_engaged_once_{false};
	uint8_t nav_state_{0};
	uint8_t arming_state_{0};

	// ---- 安全圍籬（x/y: mocap 世界座標絕對範圍；z: 相對 home 的高度限制）----
	bool fence_enabled_{true};
	double fence_world_x_min_{-1.5};
	double fence_world_x_max_{1.5};
	double fence_world_y_min_{-1.3};
	double fence_world_y_max_{1.3};
	double fence_height_max_{1.5};
	double fence_height_min_{0.0};

	// ---- setpoint 單次移動量限速 ----
	double max_horizontal_speed_mps_{0.5};
	double max_vertical_speed_mps_{0.3};

	bool has_last_setpoint_{false};
	double last_fx_{0.0}, last_fy_{0.0}, last_fz_{0.0};
	rclcpp::Time last_setpoint_time_;

	// ---- yaw 限速 ----
	double max_yaw_rate_deg_s_{30.0};
	double max_yaw_rate_rad_s_{0.5235987756};   // 30 deg/s，會在讀完參數後重新換算
	bool has_last_yaw_{false};
	double last_yaw_{0.0};
	rclcpp::Time last_yaw_time_;

	// ---- 碰撞避免 ----
	bool collision_avoidance_enabled_{true};
	double min_separation_m_{0.6};
	double collision_brake_distance_m_{0.3};
	std::vector<int64_t> collision_watch_mav_ids_;
	std::vector<std::shared_ptr<WatchedVehicle>> watched_vehicles_;

	// ---- 狀態 log ----
	double status_log_interval_ms_{2000.0};
	double near_fence_margin_m_{0.3};

	// ---- Leader 搖桿 teleop（期望速度輸入,取代「遙控器切模式搶控制權」）----
	// 設計：leader 全程留在 Offboard，不再靠 RC 切出 Offboard 來手動接管。
	// 搖桿的 pitch/roll 偏移量 -> (經 deadband) -> 機身座標系期望速度
	// -> 用固定的 yaw_ 轉世界座標 -> CBF 限制朝圍籬方向的速度分量(電子圍籬/盲區) -> 積分成位置目標。
	// z 高度全程鎖定在 home_z_ - hover_altitude_，搖桿完全不影響高度。
	bool teleop_enabled_{true};
	std::string rc_topic_suffix_{"/fmu/out/manual_control_setpoint"};
	double rc_deadband_{0.05};
	double teleop_max_speed_mps_{0.3};
	double fence_brake_distance_m_{0.35};

	double rc_pitch_{0.0}, rc_roll_{0.0}, rc_yaw_{0.0}, rc_throttle_{0.0};
	bool rc_valid_{false};
	bool rc_data_received_{false};
	rclcpp::Time last_rc_msg_time_;

	double teleop_x_{0.0}, teleop_y_{0.0};
	double target_yaw_teleop_{0.0};   // 左搖桿 yaw 積分出來的目前指揮航向（機頭方向）
	bool teleop_initialized_{false};
	bool has_last_teleop_time_{false};
	rclcpp::Time last_teleop_time_;

	// ---- Follower 專用：內部影子狀態追蹤 leader 目標 + 共用 CBF 電子圍籬 ----
	double follower_track_slowdown_radius_m_{0.3};   // 離目標多近開始減速（跟 fence_brake_distance_m 同一種形狀，只是用途不同）
	double follow_x_{0.0}, follow_y_{0.0};
	bool follow_shadow_initialized_{false};
	bool has_last_follow_time_{false};
	rclcpp::Time last_follow_time_;
	// 影子狀態目前離「這一輪編隊原始目標點」還有多遠——同時給 log_status() 顯示，
	// 也給 yaw_follow_leader 的參數驗證 callback 當作「是否已安定」的判斷依據，
	// 兩邊共用同一個數字，不會兜不起來。follower_track_slowdown_radius_m_ 當作安定門檻
	// （語意上本來就是「多近算接近目標」，不另外新增參數）。
	double follow_target_dist_{-1.0};   // -1 代表還沒開始追蹤過（尚未有意義的值）

	void declare_and_read_parameters()
	{
		this->declare_parameter<int>("mav_id", 1);
		this->declare_parameter<bool>("is_leader", true);
		this->declare_parameter<int>("leader_mav_id", 1);

		// ---- 隊形 preset（通用機制，之後加新隊形只改 yaml 不改 code）----
		// 三個平行陣列：名稱、x offset、y offset、z offset（世界座標系，單位公尺）
		// echelon 那組數值是備援/起飛前暫定用，起飛後會被「自動反推初始擺放」的結果覆蓋。
		// parallel 等明確指定的隊形則直接照這裡的數值。
		// 注意：ROS2 yaml 不接受空陣列 []，preset 至少要有一組。
		this->declare_parameter<std::vector<std::string>>("formation_preset_names",
			std::vector<std::string>{"echelon"});
		this->declare_parameter<std::vector<double>>("formation_preset_offset_x",
			std::vector<double>{-1.0});
		this->declare_parameter<std::vector<double>>("formation_preset_offset_y",
			std::vector<double>{-1.0});
		this->declare_parameter<std::vector<double>>("formation_preset_offset_z",
			std::vector<double>{0.0});

		// 目前套用哪一組 preset（runtime 可以 ros2 param set 切換）
		// 特殊值 "custom"：不查 preset 表，改成即時讀 custom_offset_x/y/z 這三個參數，
		// 可以隨時 ros2 param set 直接生效，不用切換 formation_mode 就能立刻改變 offset。
		this->declare_parameter<std::string>("formation_mode", "echelon");

		// formation_mode="custom" 時使用的即時偏移量，單位公尺，
		// x=固定座標系/leader機身座標系前方(視 yaw_follow_leader 而定，跟其他 preset 規則一致)，
		// y=右方，z 通常維持 0（三台機飛行高度應該一致）。
		this->declare_parameter<double>("custom_offset_x", 0.0);
		this->declare_parameter<double>("custom_offset_y", 0.0);
		this->declare_parameter<double>("custom_offset_z", 0.0);

		// auto_calibrate_offset: true 時，follower 在「自己位置 valid + leader 位置 valid」
		// 同時成立的第一個瞬間，用「自己當下實際位置 - leader 當下位置」反推出 active_offset，
		// 之後追蹤用這個反推值（不再用 yaml preset 的 x/y/z）。
		// 好處是不需要把機身精確擺在量測的 offset 位置，擺哪就從哪維持相對位置。
		// false 時直接用 preset yaml 的固定 offset，不做反推。
		this->declare_parameter<bool>("auto_calibrate_offset", true);

		this->declare_parameter<double>("yaw", 0.0);
		this->declare_parameter<double>("hover_altitude", 0.5);
		this->declare_parameter<std::string>("start_topic", "/formation/start");
		this->declare_parameter<double>("leader_timeout_ms", 500.0);
		// 這幾個 topic 後綴：不同 PX4/px4_msgs 版本，各訊息是否帶 "_v1" 版本後綴可能不一致
		// （每個訊息各自獨立編號，不能假設全部一致），對不上時完全沒有錯誤訊息，只會
		// 安靜地收不到資料——上機發現卡住時，先用 `ros2 topic list | grep vehicle_status`
		// / `grep vehicle_local_position` 確認實際名稱，改這裡就好，不用重編。
		this->declare_parameter<std::string>("position_topic_suffix", "/fmu/out/vehicle_local_position_v1");
		this->declare_parameter<std::string>("vehicle_status_topic_suffix", "/fmu/out/vehicle_status_v1");
		this->declare_parameter<std::string>("vehicle_command_ack_topic_suffix", "/fmu/out/vehicle_command_ack");

		// leader heading 來源："px4"（預設）或 "mocap"（繞開 EKF2 gating，只有真機能用，
		// 見 formation_offboard_control.cpp 裡 yaw_source_ 的註解）。
		this->declare_parameter<std::string>("yaw_source", "px4");
		this->declare_parameter<std::string>("mocap_odometry_topic_suffix", "/fmu/in/vehicle_visual_odometry");
		this->declare_parameter<double>("mocap_yaw_timeout_ms", 500.0);

		// ---- 位置合理性檢查 ----
		// 兩筆位置之間隱含的速度超過這個值(m/s)就判定不合理，拒絕採用。
		// 場地不大、我們自己所有指令的速度上限都遠低於這個值(teleop 0.3、follower 追蹤預設 0.5)，
		// 設 2.0 留了充分餘裕給正常動作，同時足以攔下 EKF/mocap 發散時常見的 3+ m/s 飄移。
		this->declare_parameter<double>("max_plausible_speed_mps", 2.0);
		// 連續幾次不合理才判定「這個位置來源目前不可信」，避免單一雜訊突波就誤判導致誤觸發凍結。
		this->declare_parameter<int>("position_reject_streak_limit", 3);
		// 已經被判定不可信之後，要連續幾筆都合理才真正恢復信任（比只憑「一筆」恢復更保守）。
		// 事故複盤發現：EKF/mocap 持續發散的過程中，偶爾會有單一樣本剛好落在門檻內，
		// 如果只憑一筆就恢復，反而會把還沒真正穩定下來的壞值當成新的可信基準。
		this->declare_parameter<int>("position_recovery_streak_required", 5);
		// follower 用 leader 當下速度外插「現在」位置時的固定延遲常數猜測值(秒)，
		// 現階段先設 0(不補償)，等實測知道大概延遲量再調。
		this->declare_parameter<double>("leader_link_latency_estimate_s", 0.0);

		// ---- 起飛同步「集合點(barrier)」----
		// 隊形裡「所有機」的 mav_id 清單（包含自己），每台機都要填一樣的完整清單，
		// 這樣每台機才知道要等誰的 /formation/ready 到齊。
		// 注意：跟 collision_watch_mav_ids 一樣，ROS2 yaml 不接受空陣列 []，
		// 如果不填這個 key，程式碼會 fallback 成「只等自己」(=不做多機同步，維持單機測試可用)。
		this->declare_parameter<std::vector<int64_t>>("formation_mav_ids", std::vector<int64_t>{});
		// 所有機 ready 到齊後，再等這段緩衝時間（各自本地倒數，不需要跨機時鐘同步）才真正起飛。
		this->declare_parameter<double>("liftoff_grace_period_s", 1.0);

		// ---- 第二層 barrier：等大家都真的 arm 成功，才一起真正爬升 ----
		// 第一層 barrier(上面那個)只同步「開始嘗試 arm」的時間點，PX4 內部 arm 處理耗時
		// 各機可能不一樣（事故分析發現最差差了約 8 秒）。這一層讓「已經 arm 成功」的機
		// 先停在地面原地等其他機，超過這個秒數還沒等到全部到齊，就放棄等待、自己先爬升。
		this->declare_parameter<double>("liftoff_after_arm_timeout_s", 10.0);

		// ---- 降落（/formation/land）----
		// 收到 land 後，z 降到離 home(起飛時的地面高度) 多近算「已經落地」，只是拿來印提醒 log 用，
		// 不會觸發自動 disarm。
		this->declare_parameter<double>("landed_height_margin_m", 0.05);
		// 降落目標比 home_z_ 再往下多推多少 (m)，讓 controller 持續嘗試往更低的地方降，
		// 而不是追到記錄的地面高度就停在半空中。NED z 正值=更低，這裡填正值。
		this->declare_parameter<double>("land_extra_descend_m", 0.15);
		// leader 手動 throttle 降落：接近預估地面高度時，下降速度收斂到的「爬行速度」，
		// 不是 0——避免預估地面高度不準時卡在固定高度、飛機沒有真的落地。
		this->declare_parameter<double>("landing_crawl_speed_mps", 0.05);
		// 離預估地面高度還有多少公尺開始線性減速。
		this->declare_parameter<double>("landing_slowdown_radius_m", 0.3);

		// follow_leader 是 runtime 參數：懸停測試設 false，編隊模式設 true，
		// 用 `ros2 param set <node> follow_leader true/false` 即時切換，不用重啟。
		this->declare_parameter<bool>("follow_leader", true);

		// yaw_follow_leader 是獨立的 runtime 參數：true 時，active_offset_x_/active_offset_y_ 會被當成
		// 「leader 機身座標系（前/右）」的固定量，用 leader 即時 heading 旋轉成世界座標，
		// 同時 follower 自己的 yaw 也會跟 leader 一致（剛體編隊）。
		// false 時維持舊行為：offset 是世界座標系固定值，yaw 用 yaw_ 這個固定參數。
		// 跟 follow_leader 分開控制，方便先驗證位置跟隨，再單獨開 yaw 跟隨。
		this->declare_parameter<bool>("yaw_follow_leader", true);

		// ---- 安全圍籬 ----
		// x/y：mocap 世界座標系的絕對長方體（場地正中央為原點），跟哪一台機、home 在哪裡無關。
		// z：維持相對「自己 home」的高度限制（起飛點之上/之下）。
		this->declare_parameter<bool>("fence_enabled", true);
		this->declare_parameter<double>("fence_world_x_min", -1.5);   // mocap 世界座標 x 下限 (m)
		this->declare_parameter<double>("fence_world_x_max", 1.5);    // mocap 世界座標 x 上限 (m)
		this->declare_parameter<double>("fence_world_y_min", -1.3);   // mocap 世界座標 y 下限 (m)
		this->declare_parameter<double>("fence_world_y_max", 1.3);    // mocap 世界座標 y 上限 (m)
		this->declare_parameter<double>("fence_height_max", 1.5);     // home 之上最多這麼高 (m)
		this->declare_parameter<double>("fence_height_min", 0.0);     // home 之下最多這麼低 (m)，預設不能低於起飛點

		// ---- setpoint 單次移動量限速（防止 leader 訊號跳變/模式切換造成瞬間大位移指令）----
		this->declare_parameter<double>("max_horizontal_speed_mps", 0.5);
		this->declare_parameter<double>("max_vertical_speed_mps", 0.3);

		// ---- yaw 限速 ----
		this->declare_parameter<double>("max_yaw_rate_deg_s", 30.0);

		// ---- 碰撞避免 ----
		// 主力機制是 apply_collision_cbf_to_velocity()：速度域 CBF，逼近其他機時讓「朝對方靠近」
		// 的速度分量隨水平距離平滑收斂到 0，貼到 min_separation_m 時剛好變成 0（懸停）。
		// min_separation_m 現在是純水平最小間距，不是 3D 球體半徑——垂直分層的編隊（例如
		// 故意讓兩台機飛不同高度）不受這裡影響，垂直方向的安全性交給圍籬高度硬夾
		// （clamp_to_fence）處理。avoid_collisions() 是同一個 min_separation_m 之上的
		// fallback（位置域硬推，一樣純水平），主要保護 leader 靜止懸停這種沒有速度可以限制的
		// 路徑，以及極端邊界情況；偵測到跟其他機水平距離小於 min_separation_m 時會印 WARN，
		// 並把目標點沿「其他機→自己目標點」的水平方向推開到剛好等於 min_separation_m
		// （純 pairwise 推離，不是完整的多機協調避障，機數不多時夠用）。
		this->declare_parameter<bool>("collision_avoidance_enabled", true);
		this->declare_parameter<double>("min_separation_m", 0.6);
		// CBF 碰撞避免煞車距離（水平 XY only）：離其他機的水平距離逼近 min_separation_m 之前
		// collision_brake_distance_m 這麼多公尺，就開始線性煞車朝那台機靠近的速度分量，貼到
		// dist == min_separation_m 時該方向速度剛好收斂到 0。跟 fence_brake_distance_m 是
		// 同一種線性收斂形狀，用途不同（這裡是機對機，不是機對圍籬牆）。必須小於 min_separation_m，
		// 否則還沒開始煞車就已經違反最小間距了。
		this->declare_parameter<double>("collision_brake_distance_m", 0.3);
		// 除了 leader 以外，還要監控哪些機的位置（例如另一台 follower 的 MAV id）。
		// leader 會自動用專門的 leader_position_sub_，這裡不用重複列。
		this->declare_parameter<std::vector<int64_t>>("collision_watch_mav_ids", std::vector<int64_t>{});

		// ---- 狀態 log ----
		this->declare_parameter<double>("status_log_interval_ms", 2000.0);
		this->declare_parameter<double>("near_fence_margin_m", 0.3);   // 離圍籬任一面小於這個距離就用 WARN 等級提醒

		// ---- Leader 搖桿 teleop（只對 is_leader_=true 的 node 有意義）----
		// teleop_enabled=false 時，leader 維持舊行為：原地起飛後懸停在 home，不理搖桿。
		this->declare_parameter<bool>("teleop_enabled", true);
		// 上機前務必用 `ros2 topic echo <ns>/fmu/out/manual_control_setpoint` 確認這個 topic
		// 真的有跟著搖桿變化的數值，不同 PX4/px4_msgs 版本後綴可能不同，這裡可以直接改 yaml。
		this->declare_parameter<std::string>("rc_topic_suffix", "/fmu/out/manual_control_setpoint");
		// 搖桿中立點附近的雜訊死區，|value| < deadband 視為 0（不動）
		this->declare_parameter<double>("rc_deadband", 0.05);
		// 搖桿超過死區時的固定水平速度（等速，不看推多深）(m/s)
		this->declare_parameter<double>("teleop_max_speed_mps", 0.3);
		// CBF 電子圍籬煞車距離：離圍籬任一面小於這個距離時，開始線性限制「朝那個面」的速度分量，
		// 貼到邊界(margin=0)時該方向速度剛好收斂到 0 = 懸停，飛機不會真的越出圍籬（=盲區）。
		this->declare_parameter<double>("fence_brake_distance_m", 0.35);
		// Follower 用內部影子狀態追蹤 leader+offset 目標時，離目標多近開始減速（比例式，
		// 避免快到目標時來回震盪）。跟 fence_brake_distance_m 是同一種線性收斂形狀，用途不同。
		this->declare_parameter<double>("follower_track_slowdown_radius_m", 0.3);

		mav_id_        = this->get_parameter("mav_id").as_int();
		is_leader_     = this->get_parameter("is_leader").as_bool();
		leader_mav_id_ = this->get_parameter("leader_mav_id").as_int();

		formation_mav_ids_ = this->get_parameter("formation_mav_ids").as_integer_array();
		liftoff_grace_period_s_ = this->get_parameter("liftoff_grace_period_s").as_double();
		liftoff_after_arm_timeout_s_ = this->get_parameter("liftoff_after_arm_timeout_s").as_double();
		if (formation_mav_ids_.empty()) {
			// 沒填 formation_mav_ids：fallback 成「只等自己」，等同不做多機同步起飛
			// （方便單機測試，不會因為忘了填這個參數就卡死等不到其他機）。
			formation_mav_ids_ = {static_cast<int64_t>(mav_id_)};
			RCLCPP_WARN(this->get_logger(),
				"[MAV%d] formation_mav_ids not set - falling back to self-only "
				"(no multi-vehicle liftoff sync will happen).", mav_id_);
		}

		// 載入所有隊形 preset，建立名稱→offset 的 map
		auto preset_names = this->get_parameter("formation_preset_names").as_string_array();
		auto preset_ox    = this->get_parameter("formation_preset_offset_x").as_double_array();
		auto preset_oy    = this->get_parameter("formation_preset_offset_y").as_double_array();
		auto preset_oz    = this->get_parameter("formation_preset_offset_z").as_double_array();

		if (preset_names.size() != preset_ox.size() ||
		    preset_names.size() != preset_oy.size() ||
		    preset_names.size() != preset_oz.size()) {
			RCLCPP_ERROR(this->get_logger(),
				"[MAV%d] formation_preset arrays length mismatch! names=%zu x=%zu y=%zu z=%zu",
				mav_id_, preset_names.size(), preset_ox.size(), preset_oy.size(), preset_oz.size());
		} else {
			for (size_t i = 0; i < preset_names.size(); ++i) {
				formation_presets_[preset_names[i]] = {preset_ox[i], preset_oy[i], preset_oz[i]};
				RCLCPP_INFO(this->get_logger(),
					"[MAV%d] Loaded preset '%s': offset=(%.2f, %.2f, %.2f)",
					mav_id_, preset_names[i].c_str(), preset_ox[i], preset_oy[i], preset_oz[i]);
			}
		}

		formation_mode_    = this->get_parameter("formation_mode").as_string();
		auto_calibrate_offset_ = this->get_parameter("auto_calibrate_offset").as_bool();

		// 根據目前 mode 設定初始 active_offset（反推完成前用 preset 數值當暫定）
		update_active_offset_from_preset(formation_mode_);

		yaw_            = this->get_parameter("yaw").as_double();
		hover_altitude_ = this->get_parameter("hover_altitude").as_double();
		start_topic_    = this->get_parameter("start_topic").as_string();
		leader_timeout_ms_ = this->get_parameter("leader_timeout_ms").as_double();
		leader_link_latency_estimate_s_ = this->get_parameter("leader_link_latency_estimate_s").as_double();
		position_topic_suffix_ = this->get_parameter("position_topic_suffix").as_string();
		vehicle_status_topic_suffix_ = this->get_parameter("vehicle_status_topic_suffix").as_string();
		vehicle_command_ack_topic_suffix_ = this->get_parameter("vehicle_command_ack_topic_suffix").as_string();
		yaw_source_ = this->get_parameter("yaw_source").as_string();
		mocap_odometry_topic_suffix_ = this->get_parameter("mocap_odometry_topic_suffix").as_string();
		mocap_yaw_timeout_ms_ = this->get_parameter("mocap_yaw_timeout_ms").as_double();
		max_plausible_speed_mps_ = this->get_parameter("max_plausible_speed_mps").as_double();
		position_reject_streak_limit_ = this->get_parameter("position_reject_streak_limit").as_int();
		position_recovery_streak_required_ = this->get_parameter("position_recovery_streak_required").as_int();

		fence_enabled_       = this->get_parameter("fence_enabled").as_bool();
		fence_world_x_min_   = this->get_parameter("fence_world_x_min").as_double();
		fence_world_x_max_   = this->get_parameter("fence_world_x_max").as_double();
		fence_world_y_min_   = this->get_parameter("fence_world_y_min").as_double();
		fence_world_y_max_   = this->get_parameter("fence_world_y_max").as_double();
		fence_height_max_    = this->get_parameter("fence_height_max").as_double();
		fence_height_min_    = this->get_parameter("fence_height_min").as_double();

		max_horizontal_speed_mps_ = this->get_parameter("max_horizontal_speed_mps").as_double();
		max_vertical_speed_mps_   = this->get_parameter("max_vertical_speed_mps").as_double();

		max_yaw_rate_deg_s_  = this->get_parameter("max_yaw_rate_deg_s").as_double();
		max_yaw_rate_rad_s_  = max_yaw_rate_deg_s_ * M_PI / 180.0;

		collision_avoidance_enabled_ = this->get_parameter("collision_avoidance_enabled").as_bool();
		min_separation_m_            = this->get_parameter("min_separation_m").as_double();
		collision_brake_distance_m_  = this->get_parameter("collision_brake_distance_m").as_double();
		collision_watch_mav_ids_     = this->get_parameter("collision_watch_mav_ids").as_integer_array();

		status_log_interval_ms_ = this->get_parameter("status_log_interval_ms").as_double();
		near_fence_margin_m_    = this->get_parameter("near_fence_margin_m").as_double();

		teleop_enabled_          = this->get_parameter("teleop_enabled").as_bool();
		rc_topic_suffix_         = this->get_parameter("rc_topic_suffix").as_string();
		rc_deadband_             = this->get_parameter("rc_deadband").as_double();
		teleop_max_speed_mps_    = this->get_parameter("teleop_max_speed_mps").as_double();
		fence_brake_distance_m_  = this->get_parameter("fence_brake_distance_m").as_double();
		follower_track_slowdown_radius_m_ = this->get_parameter("follower_track_slowdown_radius_m").as_double();
		landed_height_margin_m_ = this->get_parameter("landed_height_margin_m").as_double();
		land_extra_descend_m_ = this->get_parameter("land_extra_descend_m").as_double();
		landing_crawl_speed_mps_ = this->get_parameter("landing_crawl_speed_mps").as_double();
		landing_slowdown_radius_m_ = this->get_parameter("landing_slowdown_radius_m").as_double();
	}

	std::string mav_ns_prefix()
	{
		return "/MAV" + std::to_string(mav_id_);
	}

	/**
	 * 參數驗證 callback：在操作者 `ros2 param set` 的當下擋下已知不安全的切換，
	 * 拒絕原因會直接回到操作者下指令的那個 terminal（`ros2 param set` 指令本身的回應），
	 * 不需要另外去看 log 才知道發生什麼事。只攔截這裡列出的兩個參數，其餘一律放行。
	 *
	 * - formation_mode：切到一個沒有在 formation_preset_names 裡定義的名稱（且不是
	 *   "custom"）——這種情況下原本的行為是「靜默維持 active_offset 不變」，操作者的指令
	 *   完全沒有效果卻只印一條容易被忽略的 WARN，這裡直接拒絕，讓操作者當下就知道打錯字了。
	 * - yaw_follow_leader 設為 true：
	 *     (a) 當下 follow_leader 參數不是 true —— 這種情況下開 yaw_follow_leader
	 *         完全是 no-op（見 publish_trajectory_setpoint()，rotate_offset 只在
	 *         follow_leader_now 分支裡才會被判斷），直接拒絕避免操作者誤以為生效了。
	 *     (b) follower 還沒安定在編隊目標點附近（follow_target_dist_ 還沒收斂到
	 *         follower_track_slowdown_radius_m_ 之內）—— 這是實測發現最危險的隱藏陷阱：
	 *         yaw_offset_locked_ 會用「這一刻」的相對位置反推機身座標系 offset，
	 *         如果在還在收斂移動中就打開，會把過渡期的錯誤相對位置永久鎖定下來。
	 * leader 不受這兩條 yaw_follow_leader 規則限制（對 leader 而言這個參數本來就是
	 * no-op，沒有危險性，不需要攔截）。
	 */
	rcl_interfaces::msg::SetParametersResult validate_parameter_updates(
		const std::vector<rclcpp::Parameter> &params)
	{
		rcl_interfaces::msg::SetParametersResult result;
		result.successful = true;

		for (const auto &p : params) {
			if (p.get_name() == "formation_mode") {
				std::string mode = p.as_string();
				if (mode != "custom" && formation_presets_.find(mode) == formation_presets_.end()) {
					result.successful = false;
					result.reason =
						"formation_mode '" + mode + "' not found in formation_preset_names "
						"(and is not 'custom') - rejected, active_offset stays unchanged. "
						"Check spelling, or add it to the preset yaml first.";
					return result;
				}
			} else if (p.get_name() == "yaw_follow_leader" && !is_leader_ && p.as_bool()) {
				bool follow_leader_now = this->get_parameter("follow_leader").as_bool();
				if (!follow_leader_now) {
					result.successful = false;
					result.reason =
						"yaw_follow_leader rejected: follow_leader must be enabled first "
						"(yaw_follow_leader has no effect while follow_leader is false).";
					return result;
				}
				if (follow_target_dist_ < 0.0
				    || follow_target_dist_ > follower_track_slowdown_radius_m_) {
					result.successful = false;
					result.reason =
						"yaw_follow_leader rejected: follower not settled at formation target yet "
						"(dist_to_target=" + std::to_string(follow_target_dist_) +
						"m > settle_threshold=" + std::to_string(follower_track_slowdown_radius_m_) +
						"m). Wait until log shows 'follower tracking: ... ready=true', then retry.";
					return result;
				}
			}
		}

		return result;
	}

	void timer_callback()
	{
		log_status();

		// ---- 階段 0: 等待 start，並用「集合點(barrier)」讓多機同步起飛 ----
		// 設計：start 訊號一收到就先各自 capture home（這步不用同步，純本地動作）；
		// home capture 完就開始持續回報 /formation/ready；等偵測到「隊形裡所有機都 ready 了」，
		// 才開始local倒數一段緩衝時間 (liftoff_grace_period_s_)，時間到才真正進入 started_
		// （= 開始嘗試 arm+Offboard）。這樣操作者的操作方式完全不變（還是只發一次
		// /formation/start），但真正觸發起飛動作的時機，不再直接跟「收到 start 網路封包」
		// 的時間點掛鉤，而是「大家都確認到齊」之後才一起動，同步精度不再被單一次網路延遲主導。
		if (!started_) {
			if (start_requested_ && current_position_valid_ && !home_captured_) {
				home_x_ = current_x_;
				home_y_ = current_y_;
				home_z_ = current_z_;
				home_captured_ = true;

				// Leader 搖桿 teleop 的起點 = home（跟原本「原地起飛懸停在 home」的行為一致）
				teleop_x_ = home_x_;
				teleop_y_ = home_y_;
				target_yaw_teleop_ = yaw_;   // 指揮航向從固定參數 yaw_ 開始，之後由左搖桿即時轉動
				teleop_initialized_ = true;
				has_last_teleop_time_ = false;

				// 自己一定算「已經 ready」，不管 DDS 有沒有把自己發的訊息迴圈送回自己訂閱端。
				received_ready_ids_.insert(mav_id_);

				RCLCPP_INFO(
					this->get_logger(),
					"[MAV%d] Home captured at (%.2f, %.2f, %.2f). Waiting for all formation members ready...",
					mav_id_, home_x_, home_y_, home_z_
				);
			} else if (start_requested_ && !current_position_valid_) {
				RCLCPP_WARN_THROTTLE(
					this->get_logger(), *this->get_clock(), 1000,
					"[MAV%d] Start requested but no local position yet - waiting for valid position before capturing home.",
					mav_id_
				);
			}

			if (home_captured_) {
				// 持續回報 ready（不是只發一次），對抗 WiFi 偶發丟包——
				// reliable QoS 已經有重送機制，這裡再疊一層「持續廣播」更保險。
				publish_ready();

				if (!quorum_reached_ && all_formation_members_ready()) {
					quorum_reached_ = true;
					quorum_time_ = this->get_clock()->now();
					RCLCPP_INFO(
						this->get_logger(),
						"[MAV%d] All formation members ready. Liftoff in %.1fs (synchronized barrier)...",
						mav_id_, liftoff_grace_period_s_
					);
				}

				if (!quorum_reached_) {
					RCLCPP_WARN_THROTTLE(
						this->get_logger(), *this->get_clock(), 1000,
						"[MAV%d] Still waiting for other formation members ready (%s)",
						mav_id_, missing_ready_ids_str().c_str()
					);
				} else {
					double elapsed_s = (this->get_clock()->now() - quorum_time_).nanoseconds() / 1.0e9;
					if (elapsed_s >= liftoff_grace_period_s_) {
						started_ = true;
						offboard_setpoint_counter_ = 0;
						RCLCPP_INFO(
							this->get_logger(),
							"[MAV%d] Synchronized liftoff triggered. Beginning takeoff sequence.",
							mav_id_
						);
					}
				}
			}

			if (!started_) {
				return; // 完全不發送任何飛行指令，安靜待命（ready 心跳訊息不算在內）
			}
		}

		// ---- 階段 1: 起飛 + 嘗試進入 Offboard/Arm（只到成功一次為止）----
		if (!offboard_engaged_once_ && offboard_setpoint_counter_ >= 20) {
			if (offboard_setpoint_counter_ % 10 == 0) {
				publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
				arm();
				RCLCPP_INFO(this->get_logger(), "[MAV%d] Requesting Offboard mode + Arm", mav_id_);
			}
		}

		// 偵測是否已經成功進入 Offboard + Armed，成功後鎖住、不再強制介入，
		// 這樣之後遙控器要接管（切出 Offboard）不會被我們的程式一直搶回去。
		//
		// 主要依據：vehicle_status 回報的 nav_state/arming_state。
		// 備援依據：vehicle_command_ack（見建構子裡的訂閱說明）——現場實測過某些飛控
		// 韌體設定下 vehicle_status 沒有被 uXRCE-DDS 橋接出來，會導致這個判斷永遠等不到，
		// node 誤判「一直沒成功」而不斷重試，即使飛控其實已經真的 arm+進 Offboard 成功。
		// 兩種依據任一成立就採信，vehicle_status 之後如果修好了(韌體更新)也不會被排斥。
		bool status_confirms_offboard =
			(nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD
			 && arming_state_ == VehicleStatus::ARMING_STATE_ARMED);
		bool ack_confirms_offboard = (mode_ack_accepted_ && arm_ack_accepted_);

		if (!offboard_engaged_once_ && (status_confirms_offboard || ack_confirms_offboard)) {
			offboard_engaged_once_ = true;
			RCLCPP_INFO(
				this->get_logger(),
				"[MAV%d] Offboard engaged and armed (confirmed via %s). Will NOT force mode again - RC can safely take over.",
				mav_id_,
				status_confirms_offboard ? "vehicle_status" : "vehicle_command_ack fallback"
			);
		}

		// ---- 第二層 barrier：arm 成功後，先停在地面等其他機一起 arm 成功，才真正爬升 ----
		// 第一層 barrier 只同步「開始嘗試 arm」的時間點，PX4 內部 arm 處理耗時各機可能不同
		// （事故分析發現最差差了約 8 秒）。這裡讓已經 arm 成功的機先停在地面原地（hover_target_z()
		// 在還沒被授權爬升前會回傳 home_z_，不是爬升高度），廣播「我已經 armed」，
		// 等大家都到齊、或超過 liftoff_after_arm_timeout_s_ 還沒到齊就放棄等待、自己先爬升。
		if (offboard_engaged_once_ && !liftoff_authorized_) {
			if (!waiting_for_armed_quorum_) {
				waiting_for_armed_quorum_ = true;
				armed_wait_start_time_ = this->get_clock()->now();
				received_armed_ready_ids_.insert(mav_id_);
				RCLCPP_INFO(this->get_logger(),
					"[MAV%d] Armed. Holding at ground, waiting for all formation members to arm "
					"(timeout %.1fs)...", mav_id_, liftoff_after_arm_timeout_s_);
			}

			publish_armed_ready();

			if (all_formation_members_armed_ready()) {
				liftoff_authorized_ = true;
				waiting_for_armed_quorum_ = false;
				RCLCPP_INFO(this->get_logger(),
					"[MAV%d] All formation members armed. Climbing now.", mav_id_);
			} else {
				double elapsed_s = (this->get_clock()->now() - armed_wait_start_time_).nanoseconds() / 1.0e9;
				if (elapsed_s >= liftoff_after_arm_timeout_s_) {
					liftoff_authorized_ = true;
					waiting_for_armed_quorum_ = false;
					RCLCPP_WARN(this->get_logger(),
						"[MAV%d] Timed out (%.1fs) waiting for all members to arm (%s) - "
						"proceeding to climb alone.",
						mav_id_, liftoff_after_arm_timeout_s_, missing_armed_ready_ids_str().c_str());
				} else {
					RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
						"[MAV%d] Still waiting for other members to arm (%s)",
						mav_id_, missing_armed_ready_ids_str().c_str());
				}
			}
		}

		// ---- leader 斷訊警告（僅 follower + follow_leader 開啟時有意義）----
		bool follow_leader_now     = this->get_parameter("follow_leader").as_bool();
		bool yaw_follow_leader_now = this->get_parameter("yaw_follow_leader").as_bool();

		// formation_mode：每輪都讀，跟 follow_leader 一樣支援 ros2 param set 即時切換
		std::string new_mode = this->get_parameter("formation_mode").as_string();
		if (new_mode != formation_mode_) {
			formation_mode_ = new_mode;
			update_active_offset_from_preset(formation_mode_, /*lock_offset=*/true);
		}

		if (!is_leader_ && follow_leader_now && leader_position_valid_) {
			double elapsed_ms = (this->get_clock()->now() - last_leader_msg_time_).nanoseconds() / 1.0e6;

			if (elapsed_ms > leader_timeout_ms_) {
				RCLCPP_WARN_THROTTLE(
					this->get_logger(), *this->get_clock(), 1000,
					"[MAV%d] Leader position stale for %.0f ms (timeout=%.0f ms) - holding last known target.",
					mav_id_, elapsed_ms, leader_timeout_ms_
				);
			}
		}

		publish_offboard_control_mode();
		publish_trajectory_setpoint(follow_leader_now, yaw_follow_leader_now);

		offboard_setpoint_counter_++;
	}

	void arm()
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
	}
	void disarm()
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
	}

	/** 起飛同步 barrier：廣播「我已經 home captured，準備好了」*/
	void publish_ready()
	{
		std_msgs::msg::Int32 msg;
		msg.data = mav_id_;
		ready_publisher_->publish(msg);
	}

	/**
	 * 從 px4_msgs 慣例的四元數 (w,x,y,z) 萃取純 yaw 分量。用在 yaw_source_=="mocap" 時
	 * 處理 leader_mocap_odom_sub_ 收到的 vehicle_visual_odometry.q——這筆資料是
	 * mocap_px4_bridge.cpp 已經轉換過的 PX4 慣例姿態，不是原始 mocap 資料，這裡不用再
	 * 額外做任何正負號/座標系轉換，直接套標準公式即可。
	 */
	double yaw_from_quat(const std::array<float, 4> &q)
	{
		double w = q[0], x = q[1], y = q[2], z = q[3];
		return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
	}

	/**
	 * 位置合理性檢查：比較這筆新位置跟「上一筆原始值」之間隱含的速度，
	 * 超過 max_plausible_speed_mps_ 就判定不合理，回傳 false（呼叫端應該維持原本的值不變，
	 * 不要採用這筆）。連續拒絕達到 position_reject_streak_limit_ 次，才透過 became_invalid=true
	 * 通知呼叫端「這個位置來源目前已經不可信，該進入安全模式」（一次性突波不會誤觸發）。
	 *
	 * 恢復信任採取更保守的做法（事故複盤後修正）：一旦被判定不可信，不是「一筆看起來正常
	 * 就恢復」，而是要連續 position_recovery_streak_required_ 筆都合理，才透過 recovered=true
	 * 通知呼叫端恢復正常運作。原因：實測發現 EKF/mocap 持續發散的過程中，偶爾會有單一樣本
	 * 剛好落在門檻內，如果只憑一筆就恢復信任，會把還沒真正穩定下來的壞值當成新的可信基準，
	 * 導致「判定不可信→一筆恢復→又判定不可信」反覆震盪，而且每次「恢復」採信的基準
	 * 可能一步步跟著壞掉的方向偏移。這段「連續確認」期間，呼叫端仍然視為不可信
	 * （回傳 false），繼續凍結在原本的安全值。
	 *
	 * 這是刻意設計成跟「自己的位置」「leader 的位置」「被監控機的位置」共用同一份邏輯
	 * （呼叫端各自傳自己的 PositionSanityState 實例），因為三者都需要同一種保護：
	 * 上游(EKF/mocap)一旦壞掉，我們不該盲目信任、繼續拿去算 CBF/圍籬/編隊/碰撞避免目標。
	 */
	bool check_position_plausible(PositionSanityState &st, double x, double y, double z,
	                               bool &became_invalid, bool &recovered, bool &confirming_recovery)
	{
		became_invalid = false;
		recovered = false;
		confirming_recovery = false;
		rclcpp::Time now = this->get_clock()->now();

		if (!st.has_last_raw) {
			st.last_raw_x = x;
			st.last_raw_y = y;
			st.last_raw_z = z;
			st.last_raw_time = now;
			st.has_last_raw = true;
			return true;   // 第一筆沒有東西可以比較，直接接受
		}

		double dt = (now - st.last_raw_time).nanoseconds() / 1.0e9;
		double dx = x - st.last_raw_x;
		double dy = y - st.last_raw_y;
		double dz = z - st.last_raw_z;
		double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
		double implied_speed = (dt > 1.0e-3) ? (dist / dt) : 0.0;

		// 不管這筆合不合理，都要更新「上一筆原始值」，這樣下一筆是拿「連續兩筆原始資料」比較，
		// 而不是一直錨定在很久以前的舊值——這樣即使發散後變成「平滑但持續飄移」（不是單次瞬間跳動），
		// 只要飄移速度持續超標，也會每一筆都繼續被拒絕，不會因為「跟上一個被拒絕的極端值比很接近」
		// 而被誤判成合理。
		st.last_raw_x = x;
		st.last_raw_y = y;
		st.last_raw_z = z;
		st.last_raw_time = now;

		if (implied_speed > max_plausible_speed_mps_) {
			st.recovery_streak = 0;   // 中斷連續確認，之後要重新累積才能恢復
			st.reject_streak++;
			if (!st.declared_invalid && st.reject_streak >= position_reject_streak_limit_) {
				st.declared_invalid = true;
				became_invalid = true;
			}
			return false;
		}

		// 這筆跟「上一筆原始資料」相比是合理的
		if (st.declared_invalid) {
			// 已經處於「宣告不可信」狀態：需要連續多筆都合理，才真正恢復信任，
			// 在累積確認期間仍然回傳 false（呼叫端繼續凍結，不要提早恢復），
			// 但用 confirming_recovery=true 讓呼叫端知道這筆本身其實是合理的，
			// 只是還在累積確認次數，不要印成「跳動不合理」誤導判讀。
			confirming_recovery = true;
			st.recovery_streak++;
			if (st.recovery_streak >= position_recovery_streak_required_) {
				st.declared_invalid = false;
				st.reject_streak = 0;
				st.recovery_streak = 0;
				recovered = true;
				confirming_recovery = false;
				return true;
			}
			return false;
		}

		// 正常健康路徑：從沒被宣告不可信過，單純一筆合理樣本
		st.reject_streak = 0;
		return true;
	}

	/** 起飛同步 barrier：隊形裡列出的所有 mav_id 是不是都已經回報 ready 了 */
	bool all_formation_members_ready() const
	{
		for (int64_t id : formation_mav_ids_) {
			if (received_ready_ids_.find(id) == received_ready_ids_.end()) {
				return false;
			}
		}
		return true;
	}

	/** 除錯用：印出目前還在等哪些 mav_id 回報 ready，方便現場排查是哪台機沒起來 */
	std::string missing_ready_ids_str() const
	{
		std::string s = "waiting on:";
		bool any = false;
		for (int64_t id : formation_mav_ids_) {
			if (received_ready_ids_.find(id) == received_ready_ids_.end()) {
				s += " MAV" + std::to_string(id);
				any = true;
			}
		}
		return any ? s : "waiting on: (none? check logic)";
	}

	/** 第二層 barrier：廣播「我已經 arm 成功了」*/
	void publish_armed_ready()
	{
		std_msgs::msg::Int32 msg;
		msg.data = mav_id_;
		armed_ready_publisher_->publish(msg);
	}

	/** 第二層 barrier：隊形裡列出的所有 mav_id 是不是都已經回報 armed_ready 了 */
	bool all_formation_members_armed_ready() const
	{
		for (int64_t id : formation_mav_ids_) {
			if (received_armed_ready_ids_.find(id) == received_armed_ready_ids_.end()) {
				return false;
			}
		}
		return true;
	}

	/** 除錯用：印出目前還在等哪些 mav_id 回報 armed_ready */
	std::string missing_armed_ready_ids_str() const
	{
		std::string s = "waiting on:";
		bool any = false;
		for (int64_t id : formation_mav_ids_) {
			if (received_armed_ready_ids_.find(id) == received_armed_ready_ids_.end()) {
				s += " MAV" + std::to_string(id);
				any = true;
			}
		}
		return any ? s : "waiting on: (none? check logic)";
	}

	/**
	 * 是不是已經被授權可以真正爬升到 hover_altitude。
	 * 還沒授權時（已經 armed，但還在等其他機一起 arm 成功）維持在地面高度，不爬升。
	 */
	double hover_target_z() const
	{
		if (offboard_engaged_once_ && liftoff_authorized_) {
			return home_z_ - hover_altitude_;
		}
		return home_z_;   // 已經 armed 但還在等大家 / 還沒 armed：維持在地面高度
	}

	void publish_offboard_control_mode()
	{
		OffboardControlMode msg{};
		msg.position = true;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		offboard_control_mode_publisher_->publish(msg);
	}

	/**
	 * @param follow_leader_now      true: follower 追蹤 leader+offset（編隊模式）
	 *                                false: 不管是不是 follower，都用自己的 home + hover_altitude 懸停
	 *                                （懸停測試模式，或是 leader 一律走這條路）
	 * @param yaw_follow_leader_now  true: active_offset_x_/active_offset_y_ 視為 leader 機身座標系(前/右)的固定量，
	 *                                用 leader 即時 heading 旋轉成世界座標，自己的 yaw 也跟 leader 一致。
	 *                                false: offset 維持世界座標系固定值，yaw 用 yaw_ 固定參數（舊行為）。
	 *                                只有在 follow_leader_now 也是 true、且有 leader 位置時才生效。
	 *                                另外，即使這個是 true，如果 leader 目前回報的 heading 還不可靠
	 *                                （EKF2 的 heading_good_for_control=false，通常是剛開機/磁力計還沒收斂），
	 *                                也會自動退回世界座標系固定 offset，等 heading 收斂後才自動切換。
	 */
	void publish_trajectory_setpoint(bool follow_leader_now, bool yaw_follow_leader_now)
	{
		TrajectorySetpoint msg{};

		double fx, fy, fz;
		double target_yaw = yaw_;   // 預設用固定參數，符合舊行為 / leader / 懸停模式

		if (!current_position_valid_ && home_captured_) {
			// ---- 最高優先權：自己的位置合理性檢查判定不可信（見 check_position_plausible）----
			// 完全凍結，維持上一次實際送出的 setpoint 不變，不管原本是降落/編隊/teleop
			// 哪一種模式——因為 CBF/圍籬/編隊 offset 全部都要用到 current_x_/y_/z_，
			// 這些現在已經不可信，任何跟位置有關的判斷都不該繼續做。
			// 這是事故分析後新增的安全機制：見 log_status() 裡的 ERROR 提醒。
			fx = last_fx_;
			fy = last_fy_;
			fz = last_fz_;
			target_yaw = has_last_yaw_ ? last_yaw_ : yaw_;
		} else if (landing_requested_) {
			// ---- 降落：優先權最高，不管原本是 teleop / 編隊 / 懸停哪一種模式，一律凍結 x/y。
			// z 有兩種模式：
			//   - is_leader_ && teleop_enabled_：交給 leader 手動 throttle 控制下降
			//     （compute_landing_manual_descent_z()，可以真的手動壓到底，不會被固定目標卡住）。
			//   - 其他情況（follower，或 leader 沒開 teleop）：沿用舊行為，自動慢慢降到
			//     home_z_ 再往下 land_extra_descend_m_ 這麼多（NED z 正值=更低），刻意設得比
			//     記錄的地面高度更低，讓 controller 持續嘗試往下壓，而不是追到 home_z_
			//     就懸在半空中不下去（mocap 零點雜訊 / 收斂殘留誤差常見現象）。
			// 兩種模式都沿用既有的 apply_rate_limit 做平滑速度，且都不受圍籬 z 硬夾限制
			// （見 clamp_to_fence()：降落時刻意跳過 z 方向的夾定，否則永遠降不到 home 以下）。
			// 不會自動 disarm，見 log_status() 裡的落地提醒。
			double land_target_z = home_z_ + land_extra_descend_m_;
			bool manual_descent = is_leader_ && teleop_enabled_;

			if (!landing_target_captured_ && current_position_valid_) {
				landing_fx_ = current_x_;
				landing_fy_ = current_y_;
				landing_target_captured_ = true;
				if (manual_descent) {
					landing_manual_z_ = current_z_;
					has_last_landing_manual_time_ = false;
				}
				RCLCPP_INFO(this->get_logger(),
					"[MAV%d] Landing: freezing horizontal position at (%.2f, %.2f), "
					"%s (estimated ground/home_z=%.2f + land_extra_descend_m=%.2f = %.2f)",
					mav_id_, landing_fx_, landing_fy_,
					manual_descent ? "descent now controlled by leader throttle stick"
					               : "descending automatically",
					home_z_, land_extra_descend_m_, land_target_z
				);
			}
			fx = landing_target_captured_ ? landing_fx_ : home_x_;
			fy = landing_target_captured_ ? landing_fy_ : home_y_;
			if (manual_descent && landing_target_captured_) {
				fz = compute_landing_manual_descent_z(land_target_z);
			} else {
				fz = land_target_z;
			}
			target_yaw = has_last_yaw_ ? last_yaw_ : yaw_;   // 凍結轉向，不繼續跟著搖桿/leader轉
		} else if (!is_leader_ && follow_leader_now) {
			if (leader_position_valid_) {
				// formation_mode="custom" 時，active_offset_x/y/z_ 每個 tick 直接從
				// custom_offset_x/y/z 這三個參數即時讀取，改參數馬上生效，不用切模式。
				// 注意：如果 yaw_follow_leader 已經鎖定過(yaw_offset_locked_=true)，
				// 這裡改 custom_offset 不會馬上反映到飛行軌跡——旋轉分支只認鎖定當下的
				// body_offset_x_/y_，要讓新的 custom offset 生效，需要先關掉再重新開啟
				// yaw_follow_leader 一次，強制重新鎖定（原理跟切換 preset 隊形一樣）。
				if (formation_mode_ == "custom") {
					active_offset_x_ = this->get_parameter("custom_offset_x").as_double();
					active_offset_y_ = this->get_parameter("custom_offset_y").as_double();
					active_offset_z_ = this->get_parameter("custom_offset_z").as_double();
				}

				// ---- auto calibrate：第一次兩個條件都滿足時，用實際擺放位置反推 offset ----
				if (auto_calibrate_offset_ && !offset_calibrated_ && current_position_valid_) {
					active_offset_x_ = current_x_ - leader_x_;
					active_offset_y_ = current_y_ - leader_y_;
					// z offset 強制設為 0：三台機飛行高度應該一致，
					// 不用初始擺放的高度差來決定相對高度。
					// 用反推 z 的風險是：leader 手飛高度跟 follower 起飛高度不一樣，
					// 會導致 follower 目標高度在 leader 上下大幅移動，甚至鑽地板。
					active_offset_z_ = 0.0;
					offset_calibrated_ = true;
					RCLCPP_INFO(this->get_logger(),
						"[MAV%d] Auto-calibrated offset from initial placement: "
						"x=%.2f y=%.2f z=0.0(forced) | own(%.2f,%.2f,%.2f) leader(%.2f,%.2f,%.2f)",
						mav_id_,
						active_offset_x_, active_offset_y_,
						current_x_, current_y_, current_z_,
						leader_x_, leader_y_, leader_z_
					);
				}

				// yaw_source_=="mocap" 時，leader_heading_good_for_control_ 是由
				// leader_mocap_odom_sub_ 的 callback 樂觀設成 true（收到就算數），這裡補一個
				// 逾時檢查，比照 leader_data_age_s/leader_timeout_ms_ 對「位置」既有的作法——
				// mocap 斷線/遮擋太久就不再信任這個 yaw，退回固定世界座標偏移。
				if (yaw_source_ == "mocap" && leader_heading_good_for_control_) {
					double mocap_age_ms =
						(this->get_clock()->now() - last_leader_mocap_msg_time_).nanoseconds() / 1.0e6;
					if (mocap_age_ms > mocap_yaw_timeout_ms_) {
						leader_heading_good_for_control_ = false;
					}
				}

				bool rotate_offset = yaw_follow_leader_now && leader_heading_good_for_control_;

				if (yaw_follow_leader_now && !leader_heading_good_for_control_) {
					RCLCPP_WARN_THROTTLE(
						this->get_logger(), *this->get_clock(), 2000,
						"[MAV%d] yaw_follow_leader is on but leader heading not good for control yet - "
						"falling back to fixed world-frame offset until it converges.",
						mav_id_
					);
				}
				// ---- 剛體編隊旋轉基準鎖定：rotate_offset 第一次變成 true 的那一刻 ----
				// 用「當下」的實際相對位置 (current - leader) 跟「當下」的 leader heading，
				// 反推出 leader 機身座標系 (forward, right) 下的固定 offset，只做這一次。
				// 之所以要用「當下」重新量測，而不是直接拿 active_offset_x_/y_（那是在
				// 「固定座標系」下、可能早於此刻校準出來的值）去反推，是因為：
				//   - 如果 yaw_follow_leader 在校準當下就已經是 true，這裡等同立刻鎖定，行為不變；
				//   - 如果像目前的操作流程一樣，先在「位置模式」(yaw_follow_leader=false) 校準過，
				//     之後才切換開啟 yaw follow，這段期間 leader 沒有轉動的話，「當下」重新量到的
				//     相對位置其實跟固定座標系校準值是一致的，反推結果完全正確；
				//     但如果切換開啟 yaw follow 前 leader 曾經轉動過，這裡用「當下」重新量測，
				//     才能正確反映「現在」的真實相對關係，而不是套用一個已經過期、
				//     對應不到目前 heading 的旋轉基準。
				if (rotate_offset && !yaw_offset_locked_ && current_position_valid_) {
					double dx0 = current_x_ - leader_x_;
					double dy0 = current_y_ - leader_y_;
					double psi0 = leader_heading_;
					double cos_p0 = std::cos(psi0);
					double sin_p0 = std::sin(psi0);
					// R(-psi0)：固定座標系差值 -> leader 機身座標系 (forward, right)
					body_offset_x_ =  dx0 * cos_p0 + dy0 * sin_p0;
					body_offset_y_ = -dx0 * sin_p0 + dy0 * cos_p0;
					yaw_offset_locked_ = true;

					RCLCPP_INFO(this->get_logger(),
						"[MAV%d] Rigid-body yaw offset locked: body_offset=(%.2f, %.2f) "
						"at leader_heading=%.2frad | own(%.2f,%.2f) leader(%.2f,%.2f)",
						mav_id_, body_offset_x_, body_offset_y_, psi0,
						current_x_, current_y_, leader_x_, leader_y_
					);
				}

				// ---- 用 leader 當下速度外插「現在」的 leader 位置,補償通訊延遲造成的相位落後 ----
				// 延遲 = 固定猜測常數(leader_link_latency_estimate_s_，現在預設 0)
				//        + 這筆 leader 資料距離現在已經放了多久沒更新(用 follower 自己的時鐘算，
				//          不需要跨機時鐘同步)。超過 leader_timeout_ms_ 就不做外插，避免飛控斷訊時
				//          用一個很舊的速度亂外插出更誇張的錯誤位置。
				double leader_data_age_s = (this->get_clock()->now() - last_leader_msg_time_).nanoseconds() / 1.0e9;
				if (leader_data_age_s < 0.0 || leader_data_age_s * 1000.0 > leader_timeout_ms_) {
					leader_data_age_s = 0.0;
				}
				double leader_total_latency_s = leader_link_latency_estimate_s_ + leader_data_age_s;
				double leader_x_pred = leader_x_ + leader_vx_ * leader_total_latency_s;
				double leader_y_pred = leader_y_ + leader_vy_ * leader_total_latency_s;

				if (rotate_offset && yaw_offset_locked_) {
					double psi = leader_heading_;
					double cos_psi = std::cos(psi);
					double sin_psi = std::sin(psi);
					// R(psi)：leader 機身座標系固定值 -> 固定座標系，隨 leader 當下 heading 即時旋轉
					double world_offset_x = body_offset_x_ * cos_psi - body_offset_y_ * sin_psi;
					double world_offset_y = body_offset_x_ * sin_psi + body_offset_y_ * cos_psi;
					fx = leader_x_pred + world_offset_x;
					fy = leader_y_pred + world_offset_y;
					target_yaw = psi;
				} else {
					// 還沒開 yaw follow，或 heading 還沒收斂：維持固定座標系固定量（舊行為，不套旋轉）
					fx = leader_x_pred + active_offset_x_;
					fy = leader_y_pred + active_offset_y_;
				}
				fz = liftoff_authorized_ ? (leader_z_ + active_offset_z_) : hover_target_z();
			} else {
				// leader 位置還沒到：原地懸停（不用 offset 暫定，避免反推完成瞬間跳動）
				fx = home_x_;
				fy = home_y_;
				fz = hover_target_z();
			}
		} else if (is_leader_ && teleop_enabled_ && offboard_engaged_once_ && teleop_initialized_) {
			// Leader 搖桿 teleop 模式：搖桿期望速度經 CBF 電子圍籬濾波後積分成位置目標,
			// 左搖桿 yaw 積分成指揮航向,右搖桿的前/右也會跟著這個航向轉動。
			// 高度全程鎖定，不受搖桿影響。
			compute_leader_teleop_target(fx, fy, target_yaw);
			fz = hover_target_z();
		} else {
			// 懸停測試模式 / leader 尚未進 offboard / teleop 關閉：原地起飛，停在 home 正上方 hover_altitude 公尺
			fx = home_x_;
			fy = home_y_;
			fz = hover_target_z();
		}

		// ---- Follower 專用：用內部影子狀態追蹤上面算出來的「原始目標點」，並套 CBF 電子圍籬 ----
		// 不管上面是走「編隊追蹤」「leader 位置還沒到的暫時懸停」還是「懸停測試模式」，
		// follower 都統一經過這一層：原本直接把 fx,fy 當成 setpoint 送出去,
		// 現在改成「影子狀態 follow_x_/follow_y_ 用比例速度去追這個原始目標,
		// 追蹤速度一樣套 apply_fence_cbf_to_velocity」，這樣 follower 靠近圍籬時
		// 也會平滑收斂，不會只靠舊版 clamp_to_fence 硬夾。
		// leader 完全不會走到這裡（下面用 !is_leader_ 擋住）；降落時也跳過（已經是凍結住的固定
		// 目標點，不需要再經過影子狀態追蹤那一層）。
		if (!is_leader_ && !landing_requested_) {
			double raw_fx = fx, raw_fy = fy;

			// ---- 目標點圍籬夾回（事故分析後新增）----
			// CBF 電子圍籬(apply_fence_cbf_to_velocity)只看「自己目前實際位置」離牆多近去
			// 限制速度，沒辦法阻止「目標點本身」算出一個離譜到圍籬外的值——如果上游資料
			// (leader 位置)壞掉、即使已經有合理性檢查擋一層，這裡還是直接把原始目標點
			// 夾回圍籬範圍內當最後一道保險，確保 follow_x_/follow_y_ 這個內部影子狀態
			// 本身也不會無限制往外跑（不只是靠後面 clamp_to_fence 保護「送出去的 setpoint」，
			// 內部狀態本身也要保持在合理範圍，不然 leader 恢復正常後要花很久才能收斂回來）。
			raw_fx = std::clamp(raw_fx, fence_world_x_min_, fence_world_x_max_);
			raw_fy = std::clamp(raw_fy, fence_world_y_min_, fence_world_y_max_);

			track_target_with_cbf(raw_fx, raw_fy, fx, fy);
		}

		// ---- 碰撞避免：跟其他機（leader / collision_watch_mav_ids_ 列出的機）距離太近就推開 ----
		if (collision_avoidance_enabled_) {
			avoid_collisions(fx, fy, fz);
		}

		// ---- 安全圍籬：軟性 clamp 回「相對自己 home 的長方體」內 ----
		if (fence_enabled_ && home_captured_) {
			clamp_to_fence(fx, fy, fz);
		}

		// ---- 單次移動量限速：clamp 這一輪目標點跟「上一輪實際送出的 setpoint」之間的距離 ----
		apply_rate_limit(fx, fy, fz);

		// ---- yaw 限速：clamp 這一輪 yaw 目標跟「上一輪實際送出的 yaw」之間的角度差 ----
		apply_yaw_rate_limit(target_yaw);

		msg.position = {
			static_cast<float>(fx),
			static_cast<float>(fy),
			static_cast<float>(fz)
		};

		msg.yaw = static_cast<float>(target_yaw);
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		trajectory_setpoint_publisher_->publish(msg);
	}

	/**
	 * 把 (fx, fy, fz) 夾回圍籬內。
	 * x/y 用 mocap 世界座標的絕對範圍（跟 home 無關，前提是 EKF2 local frame 原點
	 * 跟 mocap 世界座標原點對得上，否則這裡夾出來的範圍會跟著偏移）。
	 * z 維持相對「自己 home」的高度限制。PX4 local position 是 NED，z 往下為正、往上為負，
	 * 所以「home 之上 fence_height_max_ 公尺」對應到 z 的下界(較負)，
	 * 「home 之下 fence_height_min_ 公尺」對應到 z 的上界(較正)。
	 *
	 * 降落時（landing_requested_）刻意跳過 z 方向的夾定：降落的整個目的就是要降到 home
	 * 高度以下（land_extra_descend_m_ 自動降落 / leader 手動 throttle 都一樣），如果這裡
	 * 還是照常把 z 夾回 z_max=home_z_+fence_height_min_（預設 fence_height_min_=0.0，也就是
	 * z_max=home_z_），降落目標會被夾回 home 高度，永遠降不下去——這是先前就存在的邏輯
	 * 矛盾。x/y 的圍籬在降落時仍然生效（降落時 x/y 本來就是凍結住的定點，維持圍籬一樣安全）。
	 */
	void clamp_to_fence(double &fx, double &fy, double &fz)
	{
		double z_min = home_z_ - fence_height_max_;   // 最高點（z 最負）
		double z_max = home_z_ + fence_height_min_;   // 最低點（z 最正，預設不能低於 home）

		double clamped_x = std::clamp(fx, fence_world_x_min_, fence_world_x_max_);
		double clamped_y = std::clamp(fy, fence_world_y_min_, fence_world_y_max_);
		double clamped_z = landing_requested_ ? fz : std::clamp(fz, z_min, z_max);

		if (clamped_x != fx || clamped_y != fy || clamped_z != fz) {
			RCLCPP_WARN_THROTTLE(
				this->get_logger(), *this->get_clock(), 1000,
				"[MAV%d] Fence clamp active: target (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)",
				mav_id_, fx, fy, fz, clamped_x, clamped_y, clamped_z
			);
		}

		fx = clamped_x;
		fy = clamped_y;
		fz = clamped_z;
	}

	/**
	 * 限制這一輪目標點跟「上一輪實際送出的 setpoint」之間的移動距離，
	 * 水平、垂直分開限速。第一次呼叫（還沒有上一輪 setpoint）時不限速，直接採用。
	 */
	void apply_rate_limit(double &fx, double &fy, double &fz)
	{
		rclcpp::Time now = this->get_clock()->now();

		if (!has_last_setpoint_) {
			last_fx_ = fx;
			last_fy_ = fy;
			last_fz_ = fz;
			last_setpoint_time_ = now;
			has_last_setpoint_ = true;
			return;
		}

		double dt = (now - last_setpoint_time_).nanoseconds() / 1.0e9;
		last_setpoint_time_ = now;

		// 保護：dt 異常（例如時間跳變、第一次 rate 太快）時退化成不限速這一輪，避免卡死
		if (dt <= 0.0 || dt > 1.0) {
			last_fx_ = fx;
			last_fy_ = fy;
			last_fz_ = fz;
			return;
		}

		double max_h_step = max_horizontal_speed_mps_ * dt;
		double max_v_step = max_vertical_speed_mps_ * dt;

		double dx = fx - last_fx_;
		double dy = fy - last_fy_;
		double dz = fz - last_fz_;

		double h_dist = std::sqrt(dx * dx + dy * dy);
		if (h_dist > max_h_step && h_dist > 1e-6) {
			double scale = max_h_step / h_dist;
			fx = last_fx_ + dx * scale;
			fy = last_fy_ + dy * scale;
		}

		if (std::fabs(dz) > max_v_step) {
			fz = last_fz_ + std::copysign(max_v_step, dz);
		}

		last_fx_ = fx;
		last_fy_ = fy;
		last_fz_ = fz;
	}

	/**
	 * 收集目前「其他機」的水平位置 (x,y)，給碰撞 CBF 跟 fallback 的 avoid_collisions() 共用。
	 * 「其他機」定義：如果自己是 follower 且 leader_position_valid_，加入 leader；
	 * 再加上 watched_vehicles_ 裡 valid==true 的每一台（都已經通過 check_position_plausible）。
	 * 只回傳水平座標：碰撞避免現在完全在 XY 平面上做，z 由圍籬另外處理，不在這裡混用。
	 */
	struct OtherPosXY { int id; double x, y; };

	std::vector<OtherPosXY> collect_other_vehicle_positions_xy() const
	{
		std::vector<OtherPosXY> others;

		if (!is_leader_ && leader_position_valid_) {
			others.push_back({leader_mav_id_, leader_x_, leader_y_});
		}
		for (const auto &wv : watched_vehicles_) {
			if (wv->valid) {
				others.push_back({wv->mav_id, wv->x, wv->y});
			}
		}
		return others;
	}

	/**
	 * CBF 電子圍籬：限制「朝圍籬邊界方向」的速度分量，讓它隨 margin（目前實際位置離該邊界
	 * 的距離）線性收斂到 0。離邊界 >= fence_brake_distance_m_ 時完全不限制；
	 * 貼到邊界 (margin=0) 時，朝那個方向的速度分量剛好變成 0 = 懸停，
	 * 飛機不可能真的被搖桿帶出圍籬（=盲區）。沿牆切線方向 / 往場地內移動的方向不受影響。
	 *
	 * 注意：margin 是用「目前實際飛行位置」current_x_/current_y_ 算的，不是用還沒送達的
	 * teleop_x_/teleop_y_ 目標點——安全約束必須綁在真實狀態上，不能被還沒追到的 setpoint騙過。
	 *
	 * 數學上這是 h(x) = margin，CBF 條件 ḣ >= -α h(x) 在 single-integrator 假設 (ẋ=v) 下
	 * 化簡出的解析解：v_toward_boundary <= (teleop_max_speed_mps_/fence_brake_distance_m_) * h(x)，
	 * 因為圍籬是軸對齊長方體，x/y 兩軸的約束彼此獨立，不需要 QP，直接逐軸 clamp 即可。
	 */
	void apply_fence_cbf_to_velocity(double &vx, double &vy)
	{
		if (fence_brake_distance_m_ <= 1e-6) {
			return;   // 煞車距離設 0 或負值視為關閉 CBF（不建議，但保留逃生門）
		}

		double margin_x_max = fence_world_x_max_ - current_x_;   // 離 +x 邊界還有多少
		double margin_x_min = current_x_ - fence_world_x_min_;   // 離 -x 邊界還有多少
		double margin_y_max = fence_world_y_max_ - current_y_;   // 離 +y 邊界還有多少
		double margin_y_min = current_y_ - fence_world_y_min_;   // 離 -y 邊界還有多少

		auto speed_cap = [this](double margin) {
			double m = std::max(margin, 0.0);   // 已經在邊界外的話（不應發生）當作 0，不允許再往外
			return teleop_max_speed_mps_ * std::clamp(m / fence_brake_distance_m_, 0.0, 1.0);
		};

		double vx_cap_pos = speed_cap(margin_x_max);   // 往 +x 方向最多允許的速度
		double vx_cap_neg = speed_cap(margin_x_min);   // 往 -x 方向最多允許的速度（正值）
		double vy_cap_pos = speed_cap(margin_y_max);
		double vy_cap_neg = speed_cap(margin_y_min);

		if (vx > 0.0) {
			vx = std::min(vx, vx_cap_pos);
		} else if (vx < 0.0) {
			vx = std::max(vx, -vx_cap_neg);
		}

		if (vy > 0.0) {
			vy = std::min(vy, vy_cap_pos);
		} else if (vy < 0.0) {
			vy = std::max(vy, -vy_cap_neg);
		}
	}

	/**
	 * CBF 碰撞避免（速度域，只在水平 XY 平面）：跟 apply_fence_cbf_to_velocity() 同一種手法，
	 * 差別是這裡限制的不是「朝固定牆面」的速度分量，而是「朝每一台其他機當下位置」的速度分量
	 * （逐台 sequential 處理，處理完一台再處理下一台）。
	 *
	 * 刻意只做水平：z 完全不讀也不改——垂直分層的編隊（例如故意讓兩台機高度錯開）不應該
	 * 被這裡誤判成快撞在一起，反之亦然；z 方向的安全性完全交給圍籬的高度硬夾
	 * （clamp_to_fence）跟 avoid_collisions() fallback（現在也已經改成水平限定）處理。
	 *
	 * 對每一台「其他機」o：
	 *   margin = 水平距離(自己, o) - min_separation_m_
	 *   沿著「自己遠離 o」方向的速度分量 v_away，若正在朝 o 靠近且靠近速度超過煞車上限，
	 *   只 clamp 掉這個徑向分量，切線方向（繞著 o 轉的分量）完全不動。
	 * 跟 apply_fence_cbf_to_velocity() 一樣是逐一 clamp、不解 QP：因為每一步只會讓「朝任一台
	 * 其他機靠近的速度」變小，不會讓任何速度變大，所以就算多台一起處理、且是 sequential
	 * 而不是聯立求解，安全性仍然成立，只是不保證是全域最省繞路的解。
	 */
	void apply_collision_cbf_to_velocity(double &vx, double &vy)
	{
		if (collision_brake_distance_m_ <= 1e-6) {
			return;   // 煞車距離設 0 或負值視為關閉這個 CBF（不建議，但保留逃生門，跟 fence 一致）
		}

		auto others = collect_other_vehicle_positions_xy();
		if (others.empty()) {
			return;
		}

		for (const auto &o : others) {
			double dx = current_x_ - o.x;
			double dy = current_y_ - o.y;
			double dist = std::sqrt(dx * dx + dy * dy);

			if (dist <= 1e-6) {
				continue;   // 完全重疊的退化情況，方向未定義，交給 avoid_collisions() fallback 處理
			}

			double ux = dx / dist;
			double uy = dy / dist;

			double margin = dist - min_separation_m_;
			double m = std::max(margin, 0.0);
			double cap = teleop_max_speed_mps_ * std::clamp(m / collision_brake_distance_m_, 0.0, 1.0);

			double v_away = vx * ux + vy * uy;   // 正值=正在遠離 o，負值=正在靠近 o

			if (v_away < -cap) {
				double delta = (-cap) - v_away;   // 只補償徑向分量
				vx += delta * ux;
				vy += delta * uy;
			}
		}
	}

	/**
	 * 把搖桿 (rc_pitch_/rc_roll_/rc_yaw_，機身座標系前/右/轉向，範圍約 -1~1) 轉成這一輪的
	 * 位置 + yaw 目標：
	 *   1. 左搖桿 yaw 軸 -> deadband -> 等速角速度指令（死區外固定用 max_yaw_rate_rad_s_，
	 *      不管推多深，跟平移邏輯一致）-> 積分成 target_yaw_teleop_（指揮航向）。
	 *   2. 右搖桿 pitch/roll -> deadband -> 方向正規化成單位向量 -> 乘上固定速度
	 *      teleop_max_speed_mps_（等速，不用偏移量比例換算，只要有推超過死區就是全速）。
	 *   3. 用「目前指揮航向」target_yaw_teleop_（不是寫死的 yaw_！）把機身座標系方向轉成世界座標，
	 *      這樣搖桿轉向之後，右搖桿的「前」會跟著新的機頭方向走，符合一般飛手手感。
	 *   4. CBF 電子圍籬濾波 + CBF 機間碰撞避免（水平 XY only）-> 積分成位置目標。
	 *
	 * 搖桿斷訊時沒有另外處理逾時歸零，見檔案內對應說明；要加保護可以用 last_rc_msg_time_ 比對。
	 */
	void compute_leader_teleop_target(double &fx, double &fy, double &target_yaw)
	{
		// ---- 1. yaw：左搖桿 -> 等速角速度（死區外=固定角速度，不管推多深）-> 積分成指揮航向 ----
		double yaw_in = (std::fabs(rc_yaw_) < rc_deadband_) ? 0.0 : rc_yaw_;
		double yaw_rate_cmd = (yaw_in > 0.0) ? max_yaw_rate_rad_s_
		                    : (yaw_in < 0.0) ? -max_yaw_rate_rad_s_
		                    : 0.0;

		rclcpp::Time now = this->get_clock()->now();
		if (!has_last_teleop_time_) {
			last_teleop_time_ = now;
			has_last_teleop_time_ = true;
		}
		double dt = (now - last_teleop_time_).nanoseconds() / 1.0e9;
		last_teleop_time_ = now;
		if (dt <= 0.0 || dt > 1.0) {
			dt = 0.1;   // 時間跳變防呆
		}

		target_yaw_teleop_ = normalize_angle(target_yaw_teleop_ + yaw_rate_cmd * dt);
		target_yaw = target_yaw_teleop_;

		// ---- 2. 平移：右搖桿 -> deadband -> 方向正規化 -> 固定速度（等速，非比例）----
		double pitch_in = (std::fabs(rc_pitch_) < rc_deadband_) ? 0.0 : rc_pitch_;
		double roll_in  = (std::fabs(rc_roll_)  < rc_deadband_) ? 0.0 : rc_roll_;

		double body_vx = 0.0, body_vy = 0.0;
		double stick_norm = std::sqrt(pitch_in * pitch_in + roll_in * roll_in);
		if (stick_norm > 1e-6) {
			body_vx = (pitch_in / stick_norm) * teleop_max_speed_mps_;   // 前方為正
			body_vy = (roll_in  / stick_norm) * teleop_max_speed_mps_;   // 右方為正
		}

		// ---- 3. 用「目前指揮航向」把機身座標系方向轉成世界座標 ----
		double cos_y = std::cos(target_yaw_teleop_);
		double sin_y = std::sin(target_yaw_teleop_);
		double world_vx = body_vx * cos_y - body_vy * sin_y;
		double world_vy = body_vx * sin_y + body_vy * cos_y;

		// ---- 4. CBF：限制朝圍籬方向的速度分量（電子圍籬 / 盲區保護）----
		apply_fence_cbf_to_velocity(world_vx, world_vy);
		// ---- 4b. CBF：限制朝其他機靠近方向的速度分量（碰撞避免，水平 XY only）----
		if (collision_avoidance_enabled_) {
			apply_collision_cbf_to_velocity(world_vx, world_vy);
		}

		// 積分成位置目標，沿用既有的 position-setpoint 架構
		// （後面還會經過 apply_rate_limit / clamp_to_fence 當最後一道保險）
		teleop_x_ += world_vx * dt;
		teleop_y_ += world_vy * dt;

		fx = teleop_x_;
		fy = teleop_y_;
	}

	/**
	 * 降落時 leader 的手動 throttle 控制（is_leader_ && teleop_enabled_ 才會呼叫）：
	 * 把 rc_throttle_（[-1,1]，正值=上升，見 ManualControlSetpoint.msg）轉成等速的垂直速度指令
	 * （跟 compute_leader_teleop_target() 的水平搖桿是同一種「死區外固定速度」設計），
	 * 積分成 landing_manual_z_ 這個影子狀態。
	 *
	 * 下降方向（throttle 桿往下）額外套用「接近預估地面高度自動減速」：離 land_target_z
	 * (home_z_ + land_extra_descend_m_) 還有 landing_slowdown_radius_m_ 以上時全速下降，
	 * 越接近這個高度下降速度線性收斂到 landing_crawl_speed_mps_——刻意不收斂到 0，因為
	 * land_target_z 只是「預估」的地面高度（mocap 零點誤差可能讓實際地面更低），如果收斂到 0
	 * 會重新變成舊版那種「卡在固定高度、飛機沒有真的落地」的問題。上升方向（想中止降落、
	 * 拉回去）不受這個減速影響，維持全速。
	 *
	 * 這個函式不受圍籬 z 硬夾限制（見 clamp_to_fence()：landing_requested_ 時跳過 z 方向），
	 * 讓你真的可以壓著 throttle 一路降到底，不會被夾回 home 高度。
	 */
	double compute_landing_manual_descent_z(double land_target_z)
	{
		double throttle_in = (std::fabs(rc_throttle_) < rc_deadband_) ? 0.0 : rc_throttle_;

		double vz = 0.0;
		if (throttle_in > 0.0) {
			vz = -max_vertical_speed_mps_;   // 正 throttle = 上升 = NED z 變小
		} else if (throttle_in < 0.0) {
			vz = max_vertical_speed_mps_;     // 負 throttle = 下降 = NED z 變大
		}

		if (vz > 0.0 && landing_slowdown_radius_m_ > 1e-6) {
			double margin = land_target_z - current_z_;   // 離預估地面還有多少（正值=還在上方）
			double m = std::clamp(margin, 0.0, landing_slowdown_radius_m_);
			double cap = landing_crawl_speed_mps_
				+ (max_vertical_speed_mps_ - landing_crawl_speed_mps_) * (m / landing_slowdown_radius_m_);
			vz = std::min(vz, cap);
		}

		rclcpp::Time now = this->get_clock()->now();
		if (!has_last_landing_manual_time_) {
			last_landing_manual_time_ = now;
			has_last_landing_manual_time_ = true;
		}
		double dt = (now - last_landing_manual_time_).nanoseconds() / 1.0e9;
		last_landing_manual_time_ = now;
		if (dt <= 0.0 || dt > 1.0) {
			dt = 0.1;   // 時間跳變防呆
		}

		landing_manual_z_ += vz * dt;
		return landing_manual_z_;
	}

	/**
	 * Follower 專用：用內部影子狀態 (follow_x_/follow_y_) 追蹤外部給的「原始目標點」
	 * (target_x_raw, target_y_raw，也就是 leader位置+offset算出來的編隊目標，或懸停模式的 home)。
	 *
	 * 追蹤速度用比例式（離目標越遠追越快，跟 leader 搖桿「等速」不同——這裡的目標點本身
	 * 會隨 leader 移動，比例式才能自然收斂、不會在快到目標時來回震盪）：
	 *   speed = max_horizontal_speed_mps_ * clamp(dist_to_target / follower_track_slowdown_radius_m_, 0, 1)
	 * 離目標 >= slowdown_radius 時全速追，越接近目標速度線性收斂到 0。
	 *
	 * 算出追蹤速度後，套用跟 leader 完全同一個 apply_fence_cbf_to_velocity()——這個函式
	 * 只看「目前實際飛行位置」current_x_/current_y_ 算圍籬 margin，跟呼叫者是 leader 還是
	 * follower 無關，所以直接重用，靠近圍籬時一樣會平滑收斂到 0（懸停在圍籬前）。
	 * 同樣也套用 apply_collision_cbf_to_velocity()（水平 XY only），靠近其他機時一樣平滑煞車。
	 *
	 * follow_x_/follow_y_ 是「影子狀態」而不是直接拿 current_x_/current_y_ 當追蹤起點，
	 * 這樣即使 PX4 內部 position controller 對實際位置的追蹤有一點延遲，我們自己這層邏輯
	 * 仍然是乾淨的開迴路積分（跟 leader teleop_x_/teleop_y_ 是同一種設計），不會被
	 * EKF/控制器雜訊影響到「這一步該給多少速度」的判斷。
	 */
	void track_target_with_cbf(double target_x_raw, double target_y_raw, double &fx, double &fy)
	{
		if (!follow_shadow_initialized_) {
			// 第一次呼叫：影子狀態從目前實際位置開始（若還沒有效位置，退而求其次用 home）
			follow_x_ = current_position_valid_ ? current_x_ : home_x_;
			follow_y_ = current_position_valid_ ? current_y_ : home_y_;
			follow_shadow_initialized_ = true;
		}

		rclcpp::Time now = this->get_clock()->now();
		if (!has_last_follow_time_) {
			last_follow_time_ = now;
			has_last_follow_time_ = true;
		}
		double dt = (now - last_follow_time_).nanoseconds() / 1.0e9;
		last_follow_time_ = now;
		if (dt <= 0.0 || dt > 1.0) {
			dt = 0.1;   // 時間跳變防呆
		}

		double err_x = target_x_raw - follow_x_;
		double err_y = target_y_raw - follow_y_;
		double dist = std::sqrt(err_x * err_x + err_y * err_y);
		follow_target_dist_ = dist;

		double v_des_x = 0.0, v_des_y = 0.0;
		if (dist > 1e-6) {
			double speed = max_horizontal_speed_mps_ *
				std::clamp(dist / std::max(follower_track_slowdown_radius_m_, 1e-6), 0.0, 1.0);
			v_des_x = (err_x / dist) * speed;
			v_des_y = (err_y / dist) * speed;
		}

		// 跟 leader teleop 共用同一個 CBF 電子圍籬濾波（只看目前實際位置，跟角色無關）
		apply_fence_cbf_to_velocity(v_des_x, v_des_y);
		// 跟 leader teleop 共用同一個 CBF 機間碰撞避免（水平 XY only）
		if (collision_avoidance_enabled_) {
			apply_collision_cbf_to_velocity(v_des_x, v_des_y);
		}

		follow_x_ += v_des_x * dt;
		follow_y_ += v_des_y * dt;

		fx = follow_x_;
		fy = follow_y_;
	}

	/**
	 * 根據 formation_mode 字串從 preset map 載入對應的 offset 到 active_offset_x/y/z_。
	 * 如果 mode 名稱找不到，印 WARN 且維持現有 active_offset 不變。
	 *
	 * @param lock_offset
	 *   false（預設，建構子啟動時呼叫用）：維持原本行為——auto_calibrate_offset_=true 時
	 *   把 offset_calibrated_ 重設回 false，讓下一輪 timer 用「啟動當下的實際擺放位置」
	 *   反推 offset，操作者不用精確測量起飛位置，隊形自然貼合當下擺放狀態。
	 *   true（`formation_mode` 透過 ros2 param set 在飛行中即時切換時用）：**直接鎖定**
	 *   preset 表定值，把 offset_calibrated_ 設成 true——這樣飛機會真的飛去 preset
	 *   指定的新相對位置，而不會被下一輪 auto-calibrate 用「切換當下還沒動的位置」蓋掉
	 *   （蓋掉的話等於切了跟沒切一樣，這是這次追出來的實際問題）。鎖定之後，leader
	 *   跟隨邏輯完全不看這個旗標，繼續照 active_offset_x_/y_ 走，跟原本的行為一致。
	 */
	void update_active_offset_from_preset(const std::string &mode, bool lock_offset = false)
	{
		if (mode == "custom") {
			// 特殊模式：不查 preset 表。active_offset_x/y/z_ 改成每個 tick 直接讀
			// custom_offset_x/y/z 這三個參數（見 publish_trajectory_setpoint 開頭），
			// 之後要改偏移量，直接 ros2 param set custom_offset_x ... 立即生效，
			// 不需要再切一次 formation_mode。
			if (auto_calibrate_offset_) {
				offset_calibrated_ = false;
			}
			yaw_offset_locked_ = false;   // 換隊形一律重置，理由同下方一般 preset 分支
			RCLCPP_INFO(this->get_logger(),
				"[MAV%d] Switched to formation_mode 'custom': active_offset now tracks "
				"custom_offset_x/y/z live - ros2 param set those to change on the fly.",
				mav_id_);
			return;
		}

		if (formation_presets_.empty()) {
			// preset map 還沒建好（例如 declare_and_read_parameters 裡陣列長度不匹配），靜默跳過
			return;
		}

		auto it = formation_presets_.find(mode);
		if (it == formation_presets_.end()) {
			// 正常操作流程下，透過 `ros2 param set` 切到未定義的 preset 名稱會先被
			// validate_parameter_updates() 硬拒絕，根本不會走到這裡——會實際觸發這條的，
			// 只剩下 yaml 裡的初始 formation_mode 本身就打錯字這種啟動期設定錯誤，
			// 這種情況下操作者的指令(或啟動設定)完全沒有效果，値得用 ERROR 等級凸顯。
			RCLCPP_ERROR(this->get_logger(),
				"[MAV%d] formation_mode '%s' not found in presets, keeping current active_offset",
				mav_id_, mode.c_str());
			return;
		}
		active_offset_x_ = it->second.x;
		active_offset_y_ = it->second.y;
		active_offset_z_ = it->second.z;

		if (lock_offset) {
			// 飛行中即時切換 preset：直接鎖定剛剛設好的 preset 表定值，跳過下一輪的
			// auto-calibrate 反推（那段只在 !offset_calibrated_ 時才會執行）。飛機會
			// 用既有的 follower 追蹤/減速邏輯（跟平常 leader 移動時一樣）自然飛向這個
			// 新的 active_offset_x_/y_，抵達後就穩定停在 preset 定義的相對位置上。
			offset_calibrated_ = true;
		} else if (auto_calibrate_offset_) {
			// 建構子啟動時呼叫：維持原行為——讓下一輪 timer 以「啟動當下的實際擺放
			// 位置」反推 XY 偏移量，操作者不用精確測量起飛位置；auto_calibrate=false
			// 時要直接套用 preset 表定值，不應該被反推覆蓋，所以不重置。
			offset_calibrated_ = false;
		}

		// yaw_offset_locked_ 不管 auto_calibrate_offset_ 是不是開著，只要換了隊形就一定要
		// 重置：body_offset_x_/y_ 是綁定在「舊隊形」的相對位置反推出來的旋轉基準，換了新
		// 隊形的 active_offset_x_/y_ 之後，舊的旋轉基準完全對不上新隊形，繼續沿用只會讓
		// 飛機停在舊隊形位置、新 preset 的偏移量被整個忽略掉（這是原本邏輯的一個瑕疵，
		// 這兩個重置條件不應該綁在一起）。
		yaw_offset_locked_ = false;

		RCLCPP_INFO(this->get_logger(),
			"[MAV%d] Switched to formation preset '%s': active_offset=(%.2f, %.2f, %.2f)%s",
			mav_id_, mode.c_str(),
			active_offset_x_, active_offset_y_, active_offset_z_,
			lock_offset ? " (locked to preset - flying to new relative position)"
			: (auto_calibrate_offset_ ? " (will auto-calibrate from current position)" : ""));
	}

	/**
	 * 碰撞避免 fallback（純 pairwise 推離，不是完整多機協調避障，水平 XY only）：
	 * 比對這一輪目標點 (fx,fy) 跟「其他機」目前位置的水平距離，小於 min_separation_m_ 就印 WARN，
	 * 並沿著「其他機→目標點」的水平方向把目標點推到剛好等於 min_separation_m_。z 完全不讀不寫，
	 * 高度安全性交給圍籬硬夾（clamp_to_fence）處理，不受這裡影響。
	 *
	 * 現在主要的避碰工作已經由 apply_collision_cbf_to_velocity() 在速度層完成，這裡降級為
	 * 最後一道防線：主要保護 leader 靜止懸停這條沒有速度可以限制的路徑（懸停目標是寫死的
	 * home_x_/home_y_，不會經過任何 CBF），以及極端邊界情況（例如第一個 tick、或多台同時
	 * 逼近時 sequential CBF 沒收斂夠快）。跑兩輪是為了緩解「推開 A 之後反而靠近 B」的情況，
	 * 機隊機數不多時夠用。
	 */
	void avoid_collisions(double &fx, double &fy, double &fz)
	{
		(void)fz;   // 碰撞避免現在純水平，高度完全不受這裡影響

		auto others = collect_other_vehicle_positions_xy();
		if (others.empty()) {
			return;
		}

		for (int pass = 0; pass < 2; ++pass) {
			for (const auto &o : others) {
				double dx = fx - o.x;
				double dy = fy - o.y;
				double dist = std::sqrt(dx * dx + dy * dy);

				if (dist < min_separation_m_) {
					RCLCPP_WARN_THROTTLE(
						this->get_logger(), *this->get_clock(), 1000,
						"[MAV%d] ** COLLISION AVOIDANCE ** too close to MAV%d (horizontal dist=%.2fm < min_separation=%.2fm), pushing target away",
						mav_id_, o.id, dist, min_separation_m_
					);

					if (dist > 1e-6) {
						double scale = min_separation_m_ / dist;
						fx = o.x + dx * scale;
						fy = o.y + dy * scale;
					} else {
						// 完全重疊的退化情況，任意挑一個方向推開，避免除以 0
						fx = o.x + min_separation_m_;
					}
				}
			}
		}
	}

	/** 把角度（弧度）normalize 到 (-pi, pi]，用來處理 yaw 限速時的角度環繞問題 */
	static double normalize_angle(double a)
	{
		return std::atan2(std::sin(a), std::cos(a));
	}

	/**
	 * 限制這一輪 yaw 目標跟「上一輪實際送出的 yaw」之間的角度差（考慮 -pi/pi 環繞）。
	 * 第一次呼叫（還沒有上一輪 yaw）時不限速，直接採用。
	 */
	void apply_yaw_rate_limit(double &target_yaw)
	{
		rclcpp::Time now = this->get_clock()->now();

		if (!has_last_yaw_) {
			last_yaw_ = target_yaw;
			last_yaw_time_ = now;
			has_last_yaw_ = true;
			return;
		}

		double dt = (now - last_yaw_time_).nanoseconds() / 1.0e9;
		last_yaw_time_ = now;

		if (dt <= 0.0 || dt > 1.0) {
			last_yaw_ = target_yaw;
			return;
		}

		double max_step = max_yaw_rate_rad_s_ * dt;
		double diff = normalize_angle(target_yaw - last_yaw_);

		if (std::fabs(diff) > max_step) {
			diff = std::copysign(max_step, diff);
		}

		target_yaw = normalize_angle(last_yaw_ + diff);
		last_yaw_ = target_yaw;
	}

	/**
	 * 週期性印出目前飛行狀態，方便在 terminal 上直接判斷：
	 * - 現在是不是在 Offboard（還是被 RC/其他模式接管了）、有沒有 armed
	 * - 自己 / leader 的 position 是不是 valid
	 * - 目前位置離圍籬六個面各自還剩多少距離，太近時用 WARN 等級提醒
	 * 用 THROTTLE 控制頻率，不會洗版。
	 */
	void log_status()
	{
		bool is_offboard = (nav_state_ == VehicleStatus::NAVIGATION_STATE_OFFBOARD);
		bool is_armed = (arming_state_ == VehicleStatus::ARMING_STATE_ARMED);

		RCLCPP_INFO_THROTTLE(
			this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
			"[MAV%d] role=%s | flight_mode=%s (nav_state=%d) | arming=%s (arming_state=%d) | offboard_engaged_once=%s",
			mav_id_,
			is_leader_ ? "LEADER" : "FOLLOWER",
			is_offboard ? "OFFBOARD" : "NOT-OFFBOARD(RC/other)",
			nav_state_,
			is_armed ? "ARMED" : "NOT-ARMED",
			arming_state_,
			offboard_engaged_once_ ? "true" : "false"
		);

		if (!offboard_engaged_once_) {
			RCLCPP_INFO_THROTTLE(
				this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
				"[MAV%d] vehicle_command_ack fallback: mode_ack_accepted=%s arm_ack_accepted=%s "
				"(如果 vehicle_status 一直卡在 NOT-OFFBOARD/NOT-ARMED 但這兩個變成 true，"
				"代表 vehicle_status 這個 topic 沒被韌體橋接出來，飛控其實已經成功了)",
				mav_id_,
				mode_ack_accepted_ ? "true" : "false",
				arm_ack_accepted_ ? "true" : "false"
			);
		}

		RCLCPP_INFO_THROTTLE(
			this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
			"[MAV%d] own_position_valid=%s pos=(%.2f, %.2f, %.2f) | home_captured=%s home=(%.2f, %.2f, %.2f)",
			mav_id_,
			current_position_valid_ ? "true" : "false",
			current_x_, current_y_, current_z_,
			home_captured_ ? "true" : "false",
			home_x_, home_y_, home_z_
		);

		if (landing_requested_) {
			double land_target_z = home_z_ + land_extra_descend_m_;
			// 跟 offboard_engaged_once_ 用同一套備援邏輯：arming_state_ 目前這場地是壞的
			// （vehicle_status 沒被韌體橋接出來，永遠卡在預設值），只看它會導致這個分支
			// 誤判成「一直沒 armed」而永遠不印 LANDED 提醒。用 arm_ack_accepted_ 一起判斷。
			// 注意：這兩個依據都是「曾經成功過」的latch，沒辦法偵測「之後被手動 disarm」，
			// 所以斷電後這個提醒仍會持續印一陣子直到你關掉這個 node，這是已知的限制，
			// 不是安全疑慮（斷電已經完成了，只是 log 還在洗）。
			bool is_armed_now = (arming_state_ == VehicleStatus::ARMING_STATE_ARMED) || arm_ack_accepted_;
			if (!is_armed_now) {
				// 從沒真的 arm 成功過，不用印 LANDED
			} else if (home_captured_ && current_position_valid_
			           && std::fabs(current_z_ - land_target_z) < landed_height_margin_m_) {
				RCLCPP_WARN_THROTTLE(
					this->get_logger(), *this->get_clock(), 2000,
					"[MAV%d] ** LANDED ** (within %.2fm of target_z=%.2f, still ARMED) - "
					"please disarm manually via RC/QGC when ready.",
					mav_id_, landed_height_margin_m_, land_target_z
				);
			} else {
				// 正常降落進度回報，不是異常狀況，用 INFO 就好，不該用 WARN 讓人誤以為有問題。
				RCLCPP_INFO_THROTTLE(
					this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
					"[MAV%d] Landing in progress... current_z=%.2f target_z=%.2f",
					mav_id_, current_z_, land_target_z
				);
			}
		}

		if (fence_enabled_ && home_captured_) {
			double z_min = home_z_ - fence_height_max_;
			double z_max = home_z_ + fence_height_min_;

			double margin_x_min = current_x_ - fence_world_x_min_;
			double margin_x_max = fence_world_x_max_ - current_x_;
			double margin_y_min = current_y_ - fence_world_y_min_;
			double margin_y_max = fence_world_y_max_ - current_y_;
			double margin_z_min = current_z_ - z_min;   // 離「最高」那一面還有多少
			double margin_z_max = z_max - current_z_;   // 離「最低/最接近地面」那一面還有多少

			double min_margin = std::min({margin_x_min, margin_x_max, margin_y_min, margin_y_max, margin_z_min, margin_z_max});

			if (min_margin < near_fence_margin_m_) {
				RCLCPP_WARN_THROTTLE(
					this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
					"[MAV%d] ** NEAR/OUTSIDE FENCE ** margins(m): x_min=%.2f x_max=%.2f y_min=%.2f y_max=%.2f z_top=%.2f z_bottom=%.2f (min=%.2f, threshold=%.2f)",
					mav_id_, margin_x_min, margin_x_max, margin_y_min, margin_y_max, margin_z_min, margin_z_max,
					min_margin, near_fence_margin_m_
				);
			} else {
				RCLCPP_INFO_THROTTLE(
					this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
					"[MAV%d] fence margins(m): x_min=%.2f x_max=%.2f y_min=%.2f y_max=%.2f z_top=%.2f z_bottom=%.2f (min=%.2f)",
					mav_id_, margin_x_min, margin_x_max, margin_y_min, margin_y_max, margin_z_min, margin_z_max,
					min_margin
				);
			}
		}

		if (is_leader_ && teleop_enabled_) {
			double rc_elapsed_ms = rc_data_received_
				? (this->get_clock()->now() - last_rc_msg_time_).nanoseconds() / 1.0e6
				: -1.0;

			RCLCPP_INFO_THROTTLE(
				this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
				"[MAV%d] TELEOP rc_data_received=%s rc_valid=%s pitch=%.2f roll=%.2f yaw=%.2f stale_ms=%.0f | "
				"teleop_target=(%.2f, %.2f) target_yaw_teleop=%.2frad fence_brake_distance_m=%.2f teleop_max_speed_mps=%.2f",
				mav_id_,
				rc_data_received_ ? "true" : "false",
				rc_valid_ ? "true" : "false",
				rc_pitch_, rc_roll_, rc_yaw_, rc_elapsed_ms,
				teleop_x_, teleop_y_, target_yaw_teleop_, fence_brake_distance_m_, teleop_max_speed_mps_
			);
		}

		if (!is_leader_) {
			double elapsed_ms = leader_position_valid_
				? (this->get_clock()->now() - last_leader_msg_time_).nanoseconds() / 1.0e6
				: -1.0;

			RCLCPP_INFO_THROTTLE(
				this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
				"[MAV%d] leader(MAV%d) pos_valid=%s pos=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f) heading=%.2frad(good=%s) stale_ms=%.0f | "
				"link_latency_est_s=%.3f | follow_leader=%s yaw_follow_leader=%s",
				mav_id_, leader_mav_id_,
				leader_position_valid_ ? "true" : "false",
				leader_x_, leader_y_, leader_z_,
				leader_vx_, leader_vy_,
				leader_heading_,
				leader_heading_good_for_control_ ? "true" : "false",
				elapsed_ms,
				leader_link_latency_estimate_s_,
				this->get_parameter("follow_leader").as_bool() ? "true" : "false",
				this->get_parameter("yaw_follow_leader").as_bool() ? "true" : "false"
			);

			RCLCPP_INFO_THROTTLE(
				this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
				"[MAV%d] formation_mode=%s | active_offset=(%.2f, %.2f, %.2f) | calibrated=%s",
				mav_id_,
				formation_mode_.c_str(),
				active_offset_x_, active_offset_y_, active_offset_z_,
				offset_calibrated_ ? "true" : "false (waiting for both positions to be valid)"
			);

			RCLCPP_INFO_THROTTLE(
				this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
				"[MAV%d] yaw_offset_locked=%s | body_offset(leader frame, forward/right)=(%.2f, %.2f)",
				mav_id_,
				yaw_offset_locked_ ? "true" : "false (waiting for yaw_follow_leader + heading_good_for_control)",
				body_offset_x_, body_offset_y_
			);

			RCLCPP_INFO_THROTTLE(
				this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
				"[MAV%d] shadow_track target_follow=(%.2f, %.2f) | slowdown_radius_m=%.2f",
				mav_id_, follow_x_, follow_y_, follower_track_slowdown_radius_m_
			);

			// follower 目前離「這一輪編隊原始目標點」還有多遠——同一個數字也是
			// yaw_follow_leader 參數驗證 callback 拿來判斷「是否已安定」的依據
			// （見 on_set_parameters_callback()），這裡印出來讓地面站 dashboard 可以解析、
			// 判斷現在能不能安全開啟 yaw_follow_leader。
			bool yaw_lock_ready = (follow_target_dist_ >= 0.0)
				&& (follow_target_dist_ <= follower_track_slowdown_radius_m_);
			RCLCPP_INFO_THROTTLE(
				this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
				"[MAV%d] follower tracking: dist_to_target=%.2fm (yaw_lock_settle_threshold=%.2fm, ready=%s)",
				mav_id_, follow_target_dist_, follower_track_slowdown_radius_m_,
				yaw_lock_ready ? "true" : "false"
			);
		}

		if (collision_avoidance_enabled_) {
			// 水平(XY)距離，跟 min_separation_m_ 現在的純水平語意保持一致（見碰撞避免相關函式的說明）
			double min_dist = -1.0;
			int min_dist_id = -1;

			for (const auto &o : collect_other_vehicle_positions_xy()) {
				double dx = current_x_ - o.x, dy = current_y_ - o.y;
				double d = std::sqrt(dx * dx + dy * dy);
				if (min_dist < 0.0 || d < min_dist) {
					min_dist = d;
					min_dist_id = o.id;
				}
			}

			if (min_dist >= 0.0) {
				if (min_dist < min_separation_m_) {
					RCLCPP_WARN_THROTTLE(
						this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
						"[MAV%d] ** TOO CLOSE ** nearest other vehicle=MAV%d horizontal_dist=%.2fm (min_separation=%.2fm)",
						mav_id_, min_dist_id, min_dist, min_separation_m_
					);
				} else {
					RCLCPP_INFO_THROTTLE(
						this->get_logger(), *this->get_clock(), static_cast<int64_t>(status_log_interval_ms_),
						"[MAV%d] nearest other vehicle=MAV%d horizontal_dist=%.2fm (min_separation=%.2fm)",
						mav_id_, min_dist_id, min_dist, min_separation_m_
					);
				}
			}
		}
	}

	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
	{
		VehicleCommand msg{};
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
		msg.param1 = param1;
		msg.param2 = param2;
		msg.command = command;
		msg.target_system = mav_id_;
		msg.target_component = 1;
		msg.source_system = mav_id_;
		msg.source_component = 1;
		msg.from_external = true;
		vehicle_command_publisher_->publish(msg);
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting formation offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}