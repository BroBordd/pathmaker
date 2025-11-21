import struct
import json
import math
import sys
import os

class Vector3:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z
    def __repr__(self): return f"({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"
    def dist_sq(self, o): return (self.x-o.x)**2 + (self.y-o.y)**2 + (self.z-o.z)**2

class PathMaker:
    def __init__(self, filepath):
        self.filepath = filepath
        self.raw_vertices = []
        self.raw_faces = [] 
        
        self.nodes = [] 
        self.adjacency = {} 
        
        # CONFIG
        self.slope_limit = 0.5  # Lower = Allows steeper ramps
        self.bridge_dist = 0.75 # Distance to force-connect gaps (Fixes U-Turns)
        self.subdivide_passes = 1 # 1 pass = 4x density. 

    def read_cob(self):
        with open(self.filepath, 'rb') as f:
            if struct.unpack('I', f.read(4))[0] != 13466: raise ValueError("Bad Magic")
            vc = struct.unpack('I', f.read(4))[0]
            fc = struct.unpack('I', f.read(4))[0]
            self.raw_vertices = [Vector3(*struct.unpack('fff', f.read(12))) for _ in range(vc)]
            self.raw_faces = [struct.unpack('III', f.read(12)) for _ in range(fc)]

    def subdivide_geometry(self):
        """
        Splits every triangle into 4 smaller triangles.
        """
        if self.subdivide_passes == 0: return
        
        new_faces = []
        midpoint_cache = {}

        def get_midpoint(i1, i2):
            key = tuple(sorted((i1, i2)))
            if key in midpoint_cache: return midpoint_cache[key]
            v1, v2 = self.raw_vertices[i1], self.raw_vertices[i2]
            mid = Vector3((v1.x+v2.x)/2, (v1.y+v2.y)/2, (v1.z+v2.z)/2)
            idx = len(self.raw_vertices)
            self.raw_vertices.append(mid)
            midpoint_cache[key] = idx
            return idx

        for f in self.raw_faces:
            v0, v1, v2 = f[0], f[1], f[2]
            a = get_midpoint(v0, v1)
            b = get_midpoint(v1, v2)
            c = get_midpoint(v2, v0)
            new_faces.extend([(v0, a, c), (v1, b, a), (v2, c, b), (a, b, c)])
            
        self.raw_faces = new_faces

    def generate_graph(self):
        face_map = {} 
        
        # 1. Create Nodes
        for i, indices in enumerate(self.raw_faces):
            v1, v2, v3 = [self.raw_vertices[x] for x in indices]
            
            # Normal Calc
            ux, uy, uz = v2.x-v1.x, v2.y-v1.y, v2.z-v1.z
            vx, vy, vz = v3.x-v1.x, v3.y-v1.y, v3.z-v1.z
            nx, ny, nz = uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx
            length = math.sqrt(nx*nx + ny*ny + nz*nz)
            if length == 0: continue
            
            if (ny/length) > self.slope_limit:
                center = Vector3((v1.x+v2.x+v3.x)/3, (v1.y+v2.y+v3.y)/3, (v1.z+v2.z+v3.z)/3)
                nid = len(self.nodes)
                self.nodes.append({'id': nid, 'p': [round(center.x, 3), round(center.y, 3), round(center.z, 3)]})
                face_map[i] = nid
                self.adjacency[nid] = []

        # 2. Link Neighbors (Fuzzy Bridging)
        grid = {}
        grid_size = 1.5
        
        for node in self.nodes:
            gx, gy, gz = int(node['p'][0]/grid_size), int(node['p'][1]/grid_size), int(node['p'][2]/grid_size)
            key = (gx, gy, gz)
            if key not in grid: grid[key] = []
            grid[key].append(node)

        MAX_DIST_SQ = self.bridge_dist ** 2
        
        for node in self.nodes:
            gx, gy, gz = int(node['p'][0]/grid_size), int(node['p'][1]/grid_size), int(node['p'][2]/grid_size)
            potential_neighbors = []
            for dx in [-1,0,1]:
                for dy in [-1,0,1]:
                    for dz in [-1,0,1]:
                        k = (gx+dx, gy+dy, gz+dz)
                        if k in grid: potential_neighbors.extend(grid[k])
            
            px, py, pz = node['p']
            for neighbor in potential_neighbors:
                if neighbor['id'] == node['id']: continue
                
                nx, ny, nz = neighbor['p']
                dist_sq = (px-nx)**2 + (py-ny)**2 + (pz-nz)**2
                
                if dist_sq < MAX_DIST_SQ:
                    v_dist = abs(py-ny)
                    if v_dist < 1.5: # Max step height
                        if neighbor['id'] not in self.adjacency[node['id']]:
                            self.adjacency[node['id']].append(neighbor['id'])
                            if neighbor['id'] not in self.adjacency: self.adjacency[neighbor['id']] = []
                            self.adjacency[neighbor['id']].append(node['id'])

    def save_json(self, output_path):
        with open(output_path, 'w') as f:
            json.dump({ "nodes": self.nodes, "adj": self.adjacency }, f)

def process_file(input_path, output_folder):
    try:
        filename = os.path.basename(input_path)
        print(f"Processing: {filename}...")
        
        pm = PathMaker(input_path)
        pm.read_cob()
        pm.subdivide_geometry()
        pm.generate_graph()
        
        out_name = os.path.splitext(filename)[0] + ".json"
        out_path = os.path.join(output_folder, out_name)
        
        pm.save_json(out_path)
        print(f"  -> Saved to {out_path} ({len(pm.nodes)} nodes)")
        return True
    except Exception as e:
        print(f"  -> FAILED: {e}")
        return False

def main():
    if len(sys.argv) < 2:
        print("Usage: python pathmaker.py <folder_or_file>")
        return

    input_arg = sys.argv[1]
    
    # Determine Input List and Output Directory
    to_process = []
    output_dir = ""
    
    if os.path.isdir(input_arg):
        # Processing a folder
        input_dir = input_arg
        output_dir = "out"
        for f in os.listdir(input_dir):
            if f.lower().endswith(".cob"):
                to_process.append(os.path.join(input_dir, f))
    elif os.path.isfile(input_arg):
        # Processing a single file
        input_dir = os.path.dirname(input_arg)
        output_dir = os.path.join(input_dir, "out") if input_dir else "out"
        to_process.append(input_arg)
    else:
        print(f"Error: {input_arg} is not a valid file or directory.")
        return

    if not to_process:
        print("No .cob files found.")
        return

    # Create 'out' folder
    if not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir)
        except OSError as e:
            print(f"Error creating output directory: {e}")
            return

    print(f"Found {len(to_process)} files. Outputting to: {output_dir}")
    print("-" * 40)

    success_count = 0
    for filepath in to_process:
        if process_file(filepath, output_dir):
            success_count += 1

    print("-" * 40)
    print(f"Done. {success_count}/{len(to_process)} files processed successfully.")

if __name__ == "__main__":
    main()
