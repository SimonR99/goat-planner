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
import {
  FaBolt, // lightning for action
  FaQuestion, // question mark for fallback
  FaRedo, // loop arrow for retry
  FaExclamation, // exclamation for inverter
  FaNetworkWired, // network for root
  FaArrowRight, // arrow for sequence
} from "react-icons/fa";

const socket = io("http://localhost:5000");

// Update the TreeNode interface
interface TreeNode {
  type: string;
  name?: string;
  nodes?: TreeNode[];
  // Retry specific
  retries?: string;
  // Action-specific parameters
  object?: string;
  location?: string;
  method?: string;
  mode?: string;
  speed?: string;
  grip_strength?: string;
  precision?: string;
  surface?: string;
  orientation?: string;
  alignment?: string;
  message?: string;
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
  const [nodes, setNodes] = useState<Node[]>([]);
  const [edges, setEdges] = useState<Edge[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [treeData, setTreeData] = useState<TreeNode | null>(null);
  const [jsonText, setJsonText] = useState("");
  const [viewMode, setViewMode] = useState<"json" | "graph">("graph");

  const transformTreeToFlow = (treeData: TreeNode) => {
    const nodes: Node[] = [];
    const edges: Edge[] = [];
    let nodeId = 0;

    const processNode = (
      node: TreeNode,
      parentId: string | null,
      level: number,
      offsetX: number,
      totalWidth: number
    ): { id: string; width: number } => {
      const currentId = `node-${nodeId++}`;
      const nodeWidth = node.nodes?.length || 1;
      const xPosition = offsetX + (nodeWidth * totalWidth) / 2;

      // Get parameters based on node type
      const getParameters = (node: TreeNode) => {
        const params: { [key: string]: any } = {};

        if (node.retries) params.retries = node.retries;

        // Add action-specific parameters
        const actionParams = [
          "object",
          "location",
          "method",
          "mode",
          "speed",
          "grip_strength",
          "precision",
          "surface",
          "orientation",
          "alignment",
          "message",
        ];

        actionParams.forEach((param) => {
          if (param in node) {
            params[param] = node[param as keyof TreeNode];
          }
        });

        return params;
      };

      // Create node
      nodes.push({
        id: currentId,
        type: "custom",
        position: {
          x: xPosition,
          y: level * 150,
        },
        data: {
          label: node.name || node.type,
          type: node.type.toLowerCase(),
          parameters: getParameters(node),
        },
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
      if (node.nodes && node.nodes.length > 0) {
        let currentOffset = offsetX;
        node.nodes.forEach((child) => {
          const childResult = processNode(
            child,
            currentId,
            level + 1,
            currentOffset,
            totalWidth
          );
          currentOffset += childResult.width * totalWidth;
        });
      }

      return { id: currentId, width: nodeWidth };
    };

    // Start processing from root
    processNode(treeData, null, 0, 0, 250);

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

      // Transform the data for graph view
      if (data) {
        const flowData = transformTreeToFlow(data);
        setNodes(flowData.nodes);
        setEdges(flowData.edges);
      }
    } catch (error) {
      console.error("Error fetching behavior tree:", error);
      setError("Failed to fetch behavior tree. Please try again later.");
    }
  };

  useEffect(() => {
    fetchBehaviorTree();

    socket.on("plan_update", (updatedTree) => {
      console.log("Received plan update:", updatedTree);
      if (validateTree(updatedTree)) {
        setTreeData(updatedTree);
        setJsonText(JSON.stringify(updatedTree, null, 2));
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

    // Check required fields
    if (!tree.type || !Array.isArray(tree.nodes)) {
      return false;
    }

    // Convert type to lowercase for comparison
    const treeType = tree.type.toLowerCase();

    // Root should be a control node
    if (!["sequence", "fallback", "retry"].includes(treeType)) {
      return false;
    }

    // Validate all nodes in the tree
    return tree.nodes.every(validateNode);
  };

  const validateNode = (node: any): boolean => {
    if (!node || typeof node !== "object") return false;

    // Check required field type
    if (!node.type) {
      return false;
    }

    // Convert type to lowercase for comparison
    const nodeType = node.type.toLowerCase();

    // Check if it's a control node
    if (["sequence", "fallback", "retry"].includes(nodeType)) {
      // Validate Retry nodes have retries field
      if (nodeType === "retry" && !node.retries) {
        return false;
      }

      // Control nodes should have child nodes
      if (!node.nodes || !Array.isArray(node.nodes)) {
        return false;
      }

      return node.nodes.every(validateNode);
    }

    return true;
  };

  const handleJsonEdit = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const newJsonText = e.target.value;
    setJsonText(newJsonText);
    try {
      const parsedJson = JSON.parse(newJsonText);
      if (validateTree(parsedJson)) {
        setTreeData(parsedJson);
        const flowData = transformTreeToFlow(parsedJson);
        setNodes(flowData.nodes);
        setEdges(flowData.edges);
      } else {
        console.error("Invalid tree structure in JSON");
      }
    } catch (error) {
      console.error("Invalid JSON:", error);
    }
  };

  const transformTreeData = (node: TreeNode): any => {
    // Get parameters based on node type
    const getParameters = (node: TreeNode) => {
      const params: { [key: string]: any } = {};

      if (node.retries) params.retries = node.retries;

      // Add action-specific parameters
      const actionParams = [
        "object",
        "location",
        "method",
        "mode",
        "speed",
        "grip_strength",
        "precision",
        "surface",
        "orientation",
        "alignment",
        "message",
      ];

      actionParams.forEach((param) => {
        if (param in node) {
          params[param] = node[param as keyof TreeNode];
        }
      });

      return params;
    };

    const transformedNode: any = {
      name: node.name,
      type: node.type,
      parameters: getParameters(node),
    };

    if (node.nodes && node.nodes.length > 0) {
      transformedNode.children = node.nodes.map((child) =>
        transformTreeData(child)
      );
    }

    return transformedNode;
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
            style={{
              minHeight: "400px",
              resize: "none",
              whiteSpace: "pre",
              overflowWrap: "normal",
              overflowX: "auto",
            }}
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
