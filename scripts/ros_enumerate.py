#!/usr/bin/env python3
"""
Enumerate all ROS topics, services, and nodes via the ROS Master XML-RPC API.
Run from Mac against the robot's ROS master over the network.

Usage: python3 ros_enumerate.py [robot_ip]
"""
import xmlrpc.client
import sys

from config import ROBOT_IP as _DEFAULT_IP, ROS_MASTER_PORT

ROBOT_IP   = sys.argv[1] if len(sys.argv) > 1 else _DEFAULT_IP
MASTER_URI = f'http://{ROBOT_IP}:{ROS_MASTER_PORT}'

def main():
    print("Connecting to ROS master at %s ..." % MASTER_URI)
    master = xmlrpc.client.ServerProxy(MASTER_URI)

    # getSystemState returns [publishers, subscribers, services]
    code, msg, state = master.getSystemState('/')
    if code != 1:
        print("getSystemState failed: %s" % msg)
        sys.exit(1)

    publishers, subscribers, services = state

    print("=" * 60)
    print("PUBLISHERS (topic -> nodes)")
    print("=" * 60)
    for topic, nodes in sorted(publishers):
        print("  %-40s %s" % (topic, ', '.join(nodes)))

    print()
    print("=" * 60)
    print("SUBSCRIBERS (topic -> nodes)")
    print("=" * 60)
    for topic, nodes in sorted(subscribers):
        print("  %-40s %s" % (topic, ', '.join(nodes)))

    print()
    print("=" * 60)
    print("SERVICES (service -> nodes)")
    print("=" * 60)
    for service, nodes in sorted(services):
        print("  %-40s %s" % (service, ', '.join(nodes)))

    # Now get topic types via getTopicTypes (if available)
    print()
    print("=" * 60)
    print("TOPIC TYPES")
    print("=" * 60)
    try:
        code, msg, topic_types = master.getTopicTypes('/')
        if code == 1:
            for topic, ttype in sorted(topic_types):
                print("  %-40s %s" % (topic, ttype))
        else:
            print("  getTopicTypes not supported: %s" % msg)
    except Exception as e:
        print("  getTopicTypes failed: %s" % e)

    # Look for anything camera/image/video related
    print()
    print("=" * 60)
    print("CAMERA/IMAGE RELATED (filtered)")
    print("=" * 60)
    keywords = ['cam', 'image', 'video', 'rgb', 'depth', 'aivi', 'vision', 'photo', 'stream', 'frame']
    all_topics = set()
    for topic, _ in publishers + subscribers:
        all_topics.add(topic)
    for service, _ in services:
        all_topics.add(service)

    found = False
    for name in sorted(all_topics):
        lower = name.lower()
        if any(kw in lower for kw in keywords):
            print("  %s" % name)
            found = True
    if not found:
        print("  (none found)")


if __name__ == '__main__':
    main()
