# Tests (no hardware needed)

* `python tests/test_workflow.py`: synthetic encounters through `workflow.py` (head-on, jet climbing per its beacon, pattern filter, ultrasonic fusion, equipped peer).
* `tests/test_two_nodes.py`: builds `collision_node.ino` for the laptop against `mock_arduino/` and runs two nodes through `tabletop.RadioRelay`.
  Build first (Linux/Mac, from the repo root):
  ```
  cd tests/mock_arduino
  for cfg in "1 1 node1" "2 1 node2eq" "2 0 node2jet"; do set -- $cfg
    sed -e "s/#define NODE_ID     1/#define NODE_ID     $1/" -e "s/#define EQUIPPED    1/#define EQUIPPED    $2/" ../../arduino/collision_node/collision_node.ino > sketch.cpp
    g++ -std=c++17 -O1 -I. -o /tmp/mock/$3 main.cpp; done
  cd ../.. && python tests/test_two_nodes.py
  ```
  Mock test hooks on stdin: `#P <deg>` sets pitch, `#K 1` presses ACK, `#W 1` turns the autopilot switch on.
