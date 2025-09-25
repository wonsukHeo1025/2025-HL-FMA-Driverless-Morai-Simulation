#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Data collection node entry point.

This is a minimal entry point that delegates all logic to modular components.
"""

import rospy
from data_collection.node.node_core import DataCollectionNodeCore
from data_collection.node.cli_handler import CLIHandler


def main():
    """Main entry point."""
    try:
        # Initialize ROS node
        rospy.init_node('data_collection_node', anonymous=True)
        
        # Get parameters
        config = {
            'control_rate': rospy.get_param('~control_rate', 50.0),
            'control_mode': rospy.get_param('~control_mode', 'throttle'),
            'interactive': rospy.get_param('~interactive', True),
        }
        
        # Create node core
        node = DataCollectionNodeCore(config)
        
        # Create CLI handler if interactive
        cli = None
        if config['interactive']:
            cli = CLIHandler(node)
            cli.start()
        
        # Spin
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("\nKeyboard interrupt received")
    finally:
        # Clean shutdown
        if node:
            node.shutdown()
        if cli:
            cli.stop()


if __name__ == '__main__':
    main()
