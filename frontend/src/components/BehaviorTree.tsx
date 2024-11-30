import React, { useState, useEffect } from "react";
import ReactFlow, {
  Node,
  Edge,
  Controls,
  Background,
  Handle,
  Position,
  NodeProps,
} from "reactflow";
import "reactflow/dist/style.css";
import io from "socket.io-client";
import dagre from "dagre";
import {
  FaBolt, // lightning for action
  FaQuestion, // question mark for fallback
  FaRedo, // loop arrow for retry
  FaExclamation, // exclamation for inverter
  FaNetworkWired, // network for root
  FaArrowRight, // arrow for sequence
} from "react-icons/fa";

const socket = io("http://localhost:5000");

// Add the TreeNode interface
interface TreeNode {
  [key: string]: any;
  root?: {
    BehaviorTree?: any;
    main_tree_to_execute?: string;
  };
  children?: TreeNode[];
  name?: string;
}

// Add this type definition
type NodeType =
  | "root"
  | "sequence"
  | "fallback"
  | "inverter"
  | "retry"
  | "action";

// Add this helper function to get the icon
const getNodeIcon = (type: NodeType) => {
  switch (type) {
    case "root":
      return <FaNetworkWired className="text-blue-600" />;
    case "sequence":
      return <FaArrowRight className="text-green-600" />;
    case "fallback":
      return <FaQuestion className="text-yellow-600" />;
    case "inverter":
      return <FaExclamation className="text-red-600" />;
    case "retry":
      return <FaRedo className="text-purple-600" />;
    case "action":
      return <FaBolt className="text-orange-600" />;
    default:
      return <FaBolt className="text-gray-600" />;
  }
};

// Custom node types
const CustomNode: React.FC<NodeProps> = ({ data }) => {
  return (
    <div
      className="px-4 py-2 shadow-md rounded-md bg-white border-2 border-gray-200"
      style={{ width: "200px", maxWidth: "200px" }}
    >
      <Handle type="target" position={Position.Top} className="w-2 h-2" />
      <div className="flex items-center gap-2">
        <div className="text-xl flex-shrink-0">
          {getNodeIcon(data.type as NodeType)}
        </div>
        <div className="text-sm font-bold break-words overflow-hidden">
          {data.label}
        </div>
      </div>
      {data.parameters && (
        <div className="mt-2 text-xs text-gray-500">
          {Object.entries(data.parameters).map(
            ([key, value]) =>
              key !== "type" && (
                <div key={key} className="break-words overflow-hidden">
                  {`${key}: ${value}`}
                </div>
              )
          )}
        </div>
      )}
      <Handle type="source" position={Position.Bottom} className="w-2 h-2" />
    </div>
  );
};

const nodeTypes = {
  custom: CustomNode,
};

const BehaviorTree: React.FC = () => {
  // Add missing state variables
  const [nodes, setNodes] = useState<Node[]>([]);
  const [edges, setEdges] = useState<Edge[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [treeData, setTreeData] = useState<TreeNode | null>(null);
  const [jsonText, setJsonText] = useState("");
  const [viewMode, setViewMode] = useState<"json" | "graph">("graph");

  const transformTreeToFlow = (treeData: any) => {
    const nodes: Node[] = [];
    const edges: Edge[] = [];
    let nodeId = 0;

    const traverseTree = (node: any, parentId: string | null) => {
      const currentId = `node-${nodeId++}`;

      // Create node
      nodes.push({
        id: currentId,
        type: "custom",
        data: {
          label: node.name || "Unknown",
          type: node.type || "action",
          parameters: {
            ...(node.attempts ? { attempts: node.attempts } : {}),
            type: node.type,
          },
        },
        position: { x: 0, y: 0 }, // Dagre will assign the correct position later
      });

      // Create edge from parent if exists
      if (parentId) {
        edges.push({
          id: `edge-${parentId}-${currentId}`,
          source: parentId,
          target: currentId,
          type: "smoothstep",
        });
      }

      // Process children
      if (node.children && node.children.length > 0) {
        node.children.forEach((child: any) => {
          traverseTree(child, currentId);
        });
      }
    };

    // Start traversal
    if (treeData.root && treeData.root.BehaviorTree) {
      traverseTree(transformTreeData(treeData.root.BehaviorTree), null);
    } else {
      traverseTree(transformTreeData(treeData), null);
    }

    // Now apply Dagre layout
    const dagreGraph = new dagre.graphlib.Graph();
    dagreGraph.setDefaultEdgeLabel(() => ({}));

    const isHorizontal = false; // For top-down layout
    dagreGraph.setGraph({ rankdir: "TB" }); // Top to Bottom

    const nodeWidth = 200;
    const nodeHeight = 100;

    nodes.forEach((node) => {
      dagreGraph.setNode(node.id, { width: nodeWidth, height: nodeHeight });
    });

    edges.forEach((edge) => {
      dagreGraph.setEdge(edge.source, edge.target);
    });

    dagre.layout(dagreGraph);

    nodes.forEach((node) => {
      const nodeWithPosition = dagreGraph.node(node.id);
      node.position = {
        x: nodeWithPosition.x - nodeWidth / 2,
        y: nodeWithPosition.y - nodeHeight / 2,
      };
      node.targetPosition = Position.Top;
      node.sourcePosition = Position.Bottom;
    });

    return { nodes, edges };
  };

  const fetchBehaviorTree = async () => {
    try {
      setError(null);
      console.log("Fetching behavior tree...");
      const response = await fetch("http://localhost:5000/api/behavior_tree");
      console.log("Response status:", response.status);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      console.log("Fetched behavior tree data:", data);
      setTreeData(data);
      setJsonText(JSON.stringify(data, null, 2));
    } catch (error) {
      console.error("Error fetching behavior tree:", error);
      setError("Failed to fetch behavior tree. Please try again later.");
    }
  };

  useEffect(() => {
    fetchBehaviorTree();

    socket.on("plan_update", (updatedTree) => {
      if (validateTree(updatedTree)) {
        const flowData = transformTreeToFlow(updatedTree);
        setNodes(flowData.nodes);
        setEdges(flowData.edges);
      } else {
        setError("Received invalid tree structure");
      }
    });

    const intervalId = setInterval(() => {
      socket.emit("request_plan_update");
    }, 1000);

    return () => {
      socket.off("plan_update");
      clearInterval(intervalId);
    };
  }, []);

  const validateTree = (tree: any): boolean => {
    if (!tree || typeof tree !== "object") return false;

    // Check root structure
    if ("root" in tree) {
      const root = tree.root;
      if (!root.main_tree_to_execute || !root.BehaviorTree) {
        return false;
      }

      // Validate BehaviorTree structure
      const behaviorTree = root.BehaviorTree;
      if (!behaviorTree.ID || behaviorTree.ID !== root.main_tree_to_execute) {
        return false;
      }

      // Recursively validate the rest of the tree
      return Object.values(behaviorTree).every((node) => {
        if (typeof node === "object" && node !== null) {
          return validateNode(node);
        }
        return true;
      });
    }

    // If not root, validate as a node
    return validateNode(tree);
  };

  const validateNode = (node: any): boolean => {
    if (!node || typeof node !== "object") return false;

    // Check if it's a valid node structure
    if ("children" in node) {
      if (!Array.isArray(node.children)) return false;
      return node.children.every(validateNode);
    }

    // Recursively validate nested objects
    return Object.values(node).every((value) => {
      if (typeof value === "object" && value !== null) {
        if (Array.isArray(value)) {
          return value.every((item) => {
            if (typeof item === "object") return validateNode(item);
            return true;
          });
        }
        return validateNode(value);
      }
      return true;
    });
  };

  const handleJsonEdit = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const newJsonText = e.target.value;
    setJsonText(newJsonText);
    try {
      const parsedJson = JSON.parse(newJsonText);
      if (validateTree(parsedJson)) {
        setTreeData(parsedJson);
      } else {
        console.error("Invalid tree structure in JSON");
      }
    } catch (error) {
      console.error("Invalid JSON:", error);
    }
  };

  const transformTreeData = (node: TreeNode): any => {
    // If this is the root node with BehaviorTree structure
    if (node.root && node.root.BehaviorTree) {
      return transformTreeData(node.root.BehaviorTree);
    }

    // Create a node object with name and children
    const transformedNode: any = {
      name: node.name || Object.keys(node)[0] || "Unknown",
      type: getNodeType(node),
      children: [],
    };

    // Add all properties as attributes
    Object.entries(node).forEach(([key, value]) => {
      if (key !== "children" && typeof value !== "object") {
        transformedNode[key] = value;
      }
    });

    // Process children
    Object.entries(node).forEach(([key, value]) => {
      if (typeof value === "object" && value !== null) {
        if (Array.isArray(value)) {
          // Handle array of children
          value.forEach((child) => {
            if (typeof child === "object") {
              transformedNode.children.push(transformTreeData(child));
            }
          });
        } else if (key !== "name" && key !== "type") {
          // Handle nested objects as children
          transformedNode.children.push(transformTreeData(value));
        }
      }
    });

    return transformedNode;
  };

  const getNodeType = (node: TreeNode): string => {
    if ("RecoveryNode" in node) return "recovery";
    if ("PipelineSequence" in node) return "sequence";
    if ("ReactiveFallback" in node) return "fallback";
    if ("RateController" in node) return "rate";
    if ("ReactiveSequence" in node) return "sequence";
    if ("RoundRobin" in node) return "round-robin";
    return "action";
  };

  if (error) {
    return <div className="text-red-600">{error}</div>;
  }

  return (
    <div className="h-full flex flex-col overflow-hidden">
      <div className="flex justify-between items-center p-4 border-b border-gray-200">
        <h2 className="text-xl font-bold">Behavior Tree</h2>
        <label className="flex items-center cursor-pointer">
          <div className="relative">
            <input
              type="checkbox"
              className="sr-only"
              checked={viewMode === "graph"}
              onChange={() =>
                setViewMode(viewMode === "json" ? "graph" : "json")
              }
            />
            <div className="block bg-gray-300 w-14 h-8 rounded-full"></div>
            <div
              className={`dot absolute left-1 top-1 bg-blue-600 w-6 h-6 rounded-full transition ${
                viewMode === "graph" ? "transform translate-x-6" : ""
              }`}
            ></div>
          </div>
          <div className="ml-3 text-gray-700 font-medium">
            {viewMode === "json" ? "JSON" : "Graph"} View
          </div>
        </label>
      </div>

      <div className="flex-grow">
        {viewMode === "json" ? (
          <textarea
            value={jsonText}
            onChange={handleJsonEdit}
            className="w-full h-full p-4 font-mono"
          />
        ) : (
          <div style={{ width: "100%", height: "800px" }}>
            <ReactFlow
              nodes={nodes}
              edges={edges}
              nodeTypes={nodeTypes}
              fitView
              fitViewOptions={{ padding: 0.2 }}
              minZoom={0.1}
              maxZoom={1.5}
              attributionPosition="bottom-left"
            >
              <Controls />
              <Background />
            </ReactFlow>
          </div>
        )}
      </div>
    </div>
  );
};

export default BehaviorTree;
