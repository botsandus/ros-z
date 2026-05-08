package rosz

/*
#include <stdlib.h>
#include "ros_z_ffi.h"
*/
import "C"
import (
	"fmt"
	"unsafe"
)

// TopicInfo describes a discovered topic
type TopicInfo struct {
	Name     string
	TypeName string
}

// PublisherQoS describes the first observed publisher's QoS for a topic.
// Empty / zero values indicate no publisher was visible at discovery time.
type PublisherQoS struct {
	Count        int            // number of publishers visible
	Reliability  QosReliability // valid only when Count > 0
	Durability   QosDurability
	History      QosHistory
	HistoryDepth int
}

// TopicInfoWithQoS pairs a discovered topic with the first observed
// publisher's QoS. PublisherQoS.Count == 0 means no publisher was visible
// at the time of the call (e.g. snapshot before liveliness propagated).
type TopicInfoWithQoS struct {
	Name      string
	TypeName  string
	Publisher PublisherQoS
}

// NodeInfo describes a discovered node
type NodeInfo struct {
	Name      string
	Namespace string
}

// GetTopicNamesAndTypes returns all topics visible in the ROS graph
func (c *Context) GetTopicNamesAndTypes() ([]TopicInfo, error) {
	if c.handle == nil {
		return nil, fmt.Errorf("context is closed")
	}

	var cTopics *C.ros_z_topic_info_t
	var count C.uintptr_t

	result := C.ros_z_graph_get_topic_names_and_types(c.handle, &cTopics, &count)
	if result != 0 {
		return nil, NewRoszError(ErrorCode(result), "failed to get topic names and types")
	}

	n := int(count)
	if n == 0 {
		return nil, nil
	}
	defer C.ros_z_graph_free_topics(cTopics, count)

	topics := make([]TopicInfo, n)
	cSlice := unsafe.Slice(cTopics, n)
	for i := 0; i < n; i++ {
		topics[i] = TopicInfo{
			Name:     C.GoString(cSlice[i].name),
			TypeName: C.GoString(cSlice[i].type_name),
		}
	}

	return topics, nil
}

// GetTopicsWithPublisherQoS returns all topics in the ROS graph, paired
// with the first observed publisher's QoS profile per topic.
func (c *Context) GetTopicsWithPublisherQoS() ([]TopicInfoWithQoS, error) {
	if c.handle == nil {
		return nil, fmt.Errorf("context is closed")
	}

	var cTopics *C.ros_z_topic_info_with_qos_t
	var count C.uintptr_t

	result := C.ros_z_graph_get_topics_with_publisher_qos(c.handle, &cTopics, &count)
	if result != 0 {
		return nil, NewRoszError(ErrorCode(result), "failed to get topics with publisher qos")
	}

	n := int(count)
	if n == 0 {
		return nil, nil
	}
	defer C.ros_z_graph_free_topics_with_qos(cTopics, count)

	topics := make([]TopicInfoWithQoS, n)
	cSlice := unsafe.Slice(cTopics, n)
	for i := 0; i < n; i++ {
		t := cSlice[i]
		ti := TopicInfoWithQoS{
			Name:     C.GoString(t.name),
			TypeName: C.GoString(t.type_name),
		}
		if t.pub_count > 0 {
			ti.Publisher = PublisherQoS{
				Count:        int(t.pub_count),
				Reliability:  QosReliability(t.pub_reliability),
				Durability:   QosDurability(t.pub_durability),
				History:      QosHistory(t.pub_history),
				HistoryDepth: int(t.pub_history_depth),
			}
		}
		topics[i] = ti
	}

	return topics, nil
}

// GetNodeNames returns all nodes visible in the ROS graph
func (c *Context) GetNodeNames() ([]NodeInfo, error) {
	if c.handle == nil {
		return nil, fmt.Errorf("context is closed")
	}

	var cNodes *C.ros_z_node_info_t
	var count C.uintptr_t

	result := C.ros_z_graph_get_node_names(c.handle, &cNodes, &count)
	if result != 0 {
		return nil, NewRoszError(ErrorCode(result), "failed to get node names")
	}

	n := int(count)
	if n == 0 {
		return nil, nil
	}
	defer C.ros_z_graph_free_nodes(cNodes, count)

	nodes := make([]NodeInfo, n)
	cSlice := unsafe.Slice(cNodes, n)
	for i := 0; i < n; i++ {
		nodes[i] = NodeInfo{
			Name:      C.GoString(cSlice[i].name),
			Namespace: C.GoString(cSlice[i].namespace_),
		}
	}

	return nodes, nil
}

// NodeExists checks if a node with the given name and namespace exists in the graph
func (c *Context) NodeExists(name, namespace string) (bool, error) {
	if c.handle == nil {
		return false, fmt.Errorf("context is closed")
	}

	cName := C.CString(name)
	defer C.free(unsafe.Pointer(cName))

	cNamespace := C.CString(namespace)
	defer C.free(unsafe.Pointer(cNamespace))

	result := C.ros_z_graph_node_exists(c.handle, cName, cNamespace)
	if result < 0 {
		return false, NewRoszError(ErrorCode(result), "failed to check node existence")
	}

	return result == 1, nil
}
