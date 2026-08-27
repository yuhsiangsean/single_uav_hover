import rclpy

from rclpy.node import Node

from std_msgs.msg import Int32


class HoverCommander(Node):

    def __init__(self):

        super().__init__('hover_commander')

        self.publisher = self.create_publisher(
            Int32,
            '/single_uav_hover/command',
            10
        )

    # ================================================================
    # Blocking input loop
    #
    # Runs in its own terminal/process, separate from
    # hover_control's node (which prints frequent status logs). That
    # keeps this prompt from being scrolled away by log spam.
    # ================================================================

    def run(self):

        prompt = '\n[Command] 1=hover(0.5m)  2=land  > '

        while rclpy.ok():

            try:
                line = input(prompt).strip()
            except EOFError:
                break

            if line not in ('1', '2'):

                print(f'Unknown command "{line}", use 1 or 2')

                continue

            msg = Int32()

            msg.data = int(line)

            self.publisher.publish(msg)

            print(f'Sent command {line}')


def main(args=None):

    rclpy.init(args=args)

    node = HoverCommander()

    try:

        node.run()

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
