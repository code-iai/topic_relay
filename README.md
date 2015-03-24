Topic Relaying
===

This tool relays specific topics between two different ROS masters.

Run it via:
```bash
$ rosrun topic_relay relay /topic http://master1:11311 http://master2:11311
```

The type of the topic doesn't matter, but should be the same on both masters. The behavior of the node is undefined if the types differ. Also, relaying only works when something was already published on *both* masters.
