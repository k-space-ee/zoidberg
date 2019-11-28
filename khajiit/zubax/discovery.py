#!/usr/bin/env python3

import uavcan, time


# Waiting until new nodes stop appearing online.
# That would mean that all nodes that are connected to the bus are now online and ready to work.
def wait_for_all_nodes_to_become_online():
    num_nodes = 0
    while True:
        node.spin(timeout=10)
        new_num_nodes = len(dynamic_node_id_allocator.get_allocation_table())
        if new_num_nodes == num_nodes and num_nodes > 1:
            break

        num_nodes = new_num_nodes


# Determining how many ESC nodes are present.
# In real use cases though the number of ESC should be obtained from elsewhere, e.g. from control mixer settings.
# There is a helper class in PyUAVCAN that allows one to automate what we're doing here,
# but we're not using it for the purposes of greater clarity of what's going on on the protocol level.
def detect_esc_nodes():
    esc_nodes = set()
    handle = node.add_handler(uavcan.equipment.esc.Status, lambda event: esc_nodes.add(event.transfer.source_node_id))
    try:
        node.spin(timeout=3)  # Collecting ESC status messages, thus determining which nodes are ESC
    finally:
        handle.remove()

    return esc_nodes


# ----- Removed: enumeration ----

if __name__ == '__main__':
    # Initializing a UAVCAN node instance.
    # In this example we're using an SLCAN adapter on the port '/dev/ttyACM0'.
    # PyUAVCAN also supports other types of adapters, refer to its docs to learn more.
    node = uavcan.make_node('/dev/ttyACM0', node_id=10, bitrate=1000000)

    # Initializing a dynamic node ID allocator.
    # This would not be necessary if the nodes were configured to use static node ID.
    node_monitor = uavcan.app.node_monitor.NodeMonitor(node)
    dynamic_node_id_allocator = uavcan.app.dynamic_node_id.CentralizedServer(node, node_monitor)

    print('Waiting for all nodes to appear online, this should take less than a minute...')
    wait_for_all_nodes_to_become_online()
    print('Online nodes:', [node_id for _, node_id in dynamic_node_id_allocator.get_allocation_table()])

    print('Detecting ESC nodes...')
    esc_nodes = detect_esc_nodes()
    print('ESC nodes:', esc_nodes)
