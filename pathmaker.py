import struct
import json
import math
import sys
import os
import argparse
from typing import List, Tuple, Dict, Set

# --- Utility Classes ---

class Vector3:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z
    
    def to_tuple(self):
        return (self.x, self.y, self.z)

# --- PathMaker Core Logic ---

class PathMaker:
    def __init__(self, filepath):
        self.filepath = filepath
        self.raw_vertices: List[Vector3] = []
        self.raw_faces: List[Tuple[int, int, int]] = [] 
        
        self.unique_vertices: List[Vector3] = [] 
        self.welded_faces: List[Tuple[int, int, int]] = [] 
        
        self.nodes: List[Dict] = [] 
        self.adjacency: Dict[int, List[int]] = {} 
        
        # Config
        self.slope_limit = 0.6 

    def read_cob(self):
        """Parses the binary COB file."""
        with open(self.filepath, 'rb') as f:
            magic = struct.unpack('I', f.read(4))[0]
            if magic != 13466: raise ValueError("Invalid COB magic number")
            
            vertex_count = struct.unpack('I', f.read(4))[0]
            face_count = struct.unpack('I', f.read(4))[0]
            
            for _ in range(vertex_count):
                self.raw_vertices.append(Vector3(*struct.unpack('fff', f.read(12))))

            for _ in range(face_count):
                self.raw_faces.append(struct.unpack('III', f.read(12)))
            # Note: We stop reading here, skipping normals/UVs as they are not needed for graph generation

    def weld_vertices(self):
        """Merges vertices that are in the same position to connect mesh chunks."""
        vertex_map: Dict[Tuple[float, float, float], int] = {} 
        old_to_new_indices: Dict[int, int] = {}
        
        for i, v in enumerate(self.raw_vertices):
            # Key uses rounded values to handle float inaccuracies
            key = (round(v.x, 3), round(v.y, 3), round(v.z, 3))
            
            if key not in vertex_map:
                new_idx = len(self.unique_vertices)
                self.unique_vertices.append(v)
                vertex_map[key] = new_idx
            
            old_to_new_indices[i] = vertex_map[key]
            
        # Rebuild faces with welded indices
        for f in self.raw_faces:
            new_face = (
                old_to_new_indices[f[0]],
                old_to_new_indices[f[1]],
                old_to_new_indices[f[2]]
            )
            # Add only non-degenerate triangles
            if new_face[0] != new_face[1] and new_face[1] != new_face[2]:
                self.welded_faces.append(new_face)
                
    def generate_graph(self):
        """Builds the navigation graph using walkable face centroids as nodes."""
        face_map: Dict[int, int] = {}
        
        for i, indices in enumerate(self.welded_faces):
            v1, v2, v3 = (self.unique_vertices[idx] for idx in indices)
            
            # Cross product for Normal calculation (v2-v1) x (v3-v1)
            ux, uy, uz = v2.x - v1.x, v2.y - v1.y, v2.z - v1.z
            vx, vy, vz = v3.x - v1.x, v3.y - v1.y, v3.z - v1.z
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            
            length = math.sqrt(nx*nx + ny*ny + nz*nz)
            if length == 0: continue
            ny /= length
            
            # Check slope (Walkability: ny > 0.6)
            if ny > self.slope_limit:
                center = Vector3(
                    (v1.x + v2.x + v3.x) / 3.0,
                    (v1.y + v2.y + v3.y) / 3.0,
                    (v1.z + v2.z + v3.z) / 3.0
                )
                node_idx = len(self.nodes)
                self.nodes.append({
                    'id': node_idx,
                    'p': [round(center.x, 3), round(center.y, 3), round(center.z, 3)]
                })
                face_map[i] = node_idx
                self.adjacency[node_idx] = []

        # Build Connections (Adjacency List from shared edges)
        edge_to_nodes: Dict[Tuple[int, int], List[int]] = {}
        
        for face_idx, node_idx in face_map.items():
            indices = self.welded_faces[face_idx]
            edges = [
                tuple(sorted((indices[0], indices[1]))),
                tuple(sorted((indices[1], indices[2]))),
                tuple(sorted((indices[2], indices[0])))
            ]
            for edge in edges:
                if edge not in edge_to_nodes: edge_to_nodes[edge] = []
                edge_to_nodes[edge].append(node_idx)

        for nodes in edge_to_nodes.values():
            if len(nodes) > 1:
                for i in range(len(nodes)):
                    for j in range(i + 1, len(nodes)):
                        u, v = nodes[i], nodes[j]
                        if v not in self.adjacency[u]:
                            self.adjacency[u].append(v)
                            self.adjacency[v].append(u)
                            
    def save_json(self, output_path):
        """Writes the graph to a JSON file."""
        data = { "nodes": self.nodes, "adj": self.adjacency }
        with open(output_path, 'w') as f:
            json.dump(data, f)

# --- Main Execution ---

def process_file(cob_path: str, output_dir: str):
    """Handles the processing of a single COB file."""
    try:
        filename = os.path.basename(cob_path)
        json_filename = filename.replace('.cob', '.json')
        output_path = os.path.join(output_dir, json_filename)
        
        print(f"--- Processing: {filename} ---")
        
        pm = PathMaker(cob_path)
        pm.read_cob()
        pm.weld_vertices()
        pm.generate_graph()
        pm.save_json(output_path)
        
        print(f"SUCCESS: Saved to {output_path}")
        return True

    except Exception as e:
        print(f"ERROR: Failed to process {cob_path}. Reason: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description="PathMaker V3: Batch converts all .cob files in a folder to navigation graph JSON files.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "input_folder",
        type=str,
        help="Folder containing .cob collision mesh files to convert"
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default="out",
        help="Output folder name for JSON files (default: 'out')"
    )
    args = parser.parse_args()

    input_dir = args.input_folder
    output_dir = args.output

    # Validate input folder
    if not os.path.isdir(input_dir):
        print(f"ERROR: Input folder not found: '{input_dir}'")
        sys.exit(1)

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    print(f"Input folder: {input_dir}")
    print(f"Output folder: {output_dir}\n")

    # Find all .cob files
    cob_files = [
        os.path.join(input_dir, f) 
        for f in os.listdir(input_dir) 
        if f.lower().endswith('.cob')
    ]
    
    if not cob_files:
        print(f"No .cob files found in: {input_dir}")
        sys.exit(0)

    print(f"Found {len(cob_files)} .cob file(s) to process.\n")

    # Process all files
    success_count = 0
    for cob_path in cob_files:
        if process_file(cob_path, output_dir):
            success_count += 1
        print()
        
    print("=" * 50)
    print(f"Batch Processing Complete: {success_count}/{len(cob_files)} files converted successfully")
    print(f"Output location: {os.path.abspath(output_dir)}")
    print("=" * 50)

if __name__ == "__main__":
    main()
