Topic Relaying
===

This tool relays specific topics between two different ROS masters.

Run it via:
```bash
$ rosrun topic_relay relay /topic http://master1:11311 http://master2:11311
```

The type of the topic doesn't matter, but should be the same on both
masters. The behavior of the node is undefined if the types
differ. Also, relaying only works when something was already published
on *both* masters.

If on one master, the topic was not yet published, `topic_relay` will
wait until it got published. In this state, even when the other master
already got its topic, no messages will be forwarded. The moment both
topics are available (i.e., were published on at least once), the
control flow works normally.

Note that the message type on the relayed topic must be installed on
the machine `topic_relay` runs on (as it calls `rosmsg md5` in order
to find the MD5 sum for proper communication with the master).