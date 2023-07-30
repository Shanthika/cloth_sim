import os 
import open3d as o3d
import numpy as np
from pdb import set_trace
import trimesh
def init_positions(positions):
    n= positions.shape[0]
    for i in range(n):
        for j in range(n):
            positions[i, j] = np.array([i/n,j/n,0.0])
    return positions



def get_faces(n,faces):
    face_count = 0
    for i in range(n-1):
        for j in range(n-1):
            #first triangle
            f1 = i*n+j
            f2 = i*n+j+1
            f3 = ((i+1)*n)+j
            faces[face_count] = np.array([f1,f3,f2])
            face_count+=1
            # second triangle
            f4 = i*n+j+1
            f5 = ((i+1)*n)+j
            f6 = ((i+1)*n)+j+1
            faces[face_count] = np.array([f4,f5,f6])
            face_count+=1
    return faces


def update_forces(positions, velocity, gravity,dt,spring_K, spacing,damping):
    n = positions.shape[0]
    
    #v_n+1  = v_n + dv
    for i in range(n):
        for j in range(n):
            velocity[i,j] += gravity*dt
    
    #force += K*(||x_i-x_j||-L)((x_i-x_j)/||x_i-x_j||)
    
    for i in range(n):
        for j in range(n):
            force = np.zeros((3))
            # pinned vertices 
            if((i==0 and j==n-1) or (i==n-1 and j==n-1)):
                pass
            else:
                
                for k in range(-1,2):
                    for l in range(-1,2):
                        #self
                        if (k==0 and l==0) :
                            pass
                    
                        else:
                            r,c = i+k, j+l
                            if((r>=0) and (r<n) and (c>=0) and (c<n) ):
                                dist_vec = positions[i,j] - positions[r,c]
                                vel_vec = velocity[i,j] - velocity[r,c]
                                euclid_dist = np.linalg.norm(dist_vec)
                                dist_vec_norm = dist_vec/euclid_dist
                                rest_dist = np.sqrt(((i-r)**2)+((j-c)**2)) *spacing 

                                force += -spring_K* (euclid_dist-rest_dist) * (dist_vec/euclid_dist)
                                
                                force += -damping * np.dot(vel_vec,dist_vec) * dist_vec * spacing

        
                #x_n+1 = x_n + dx
                #dx = dt * dv
                velocity[i,j] += force*dt
                positions[i,j] += velocity[i,j]*dt  
            
    return positions, velocity
    
def update_vertices(vertices, positions):
    n = positions.shape[0]
    for i in range(n):
        for j in range(n):
            vertices[i*n +  j] = positions[i,j]
    
    return vertices
    
    
    
    
if __name__=='__main__':
    
    n = 8 #power of 2
    spacing = 1.0 /n
    dt = 4e-3 
    time_steps =  1000
    gravity = np.array([0,-0.098,0])
    spring_K = 50
    damping = 1e2
    #add spacing value; currently set to 1
    
    #initialize positions and velocity
    positions = np.zeros((n,n,3))
    velocity = np.zeros((n,n,3))
    
    pinning_verts = np.array([0,n-1])
    positions = init_positions(positions)
    
    
    #initialize mesh
    vertices = np.zeros((n*n,3))
    num_faces = (n-1)*(n-1)*2
    faces = np.zeros((num_faces,3))
    
    vertices = update_vertices(vertices,positions)
    faces = get_faces(n,faces)
    
    mesh_vertices = o3d.utility.Vector3dVector(vertices)
    mesh_triangles = o3d.utility.Vector3iVector(faces)
    mesh = o3d.geometry.TriangleMesh(mesh_vertices,mesh_triangles)
    
    #initialise visualizer:
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.get_render_option().mesh_show_back_face = True
    vis.get_render_option().mesh_show_wireframe = True
    vis.add_geometry(mesh)
    
    save_anim = True 
    
    path_to_save = './meshes'
    if save_anim:
        for filename in os.listdir(path_to_save):
            # Check if the file name follows the required format
            b1 = filename.startswith("frame") and filename.endswith(".png")
            b2 = filename.startswith("output.mp4")
            if b1 or b2:
                os.remove(os.path.join(path_to_save, filename))
                print('Deleted frame ' + filename)


    
    for i in range(time_steps):
        positions, velocity = update_forces(positions, velocity, gravity,dt,spring_K,spacing,damping)
    
        vertices = update_vertices(vertices,positions)
        mesh.vertices =  o3d.utility.Vector3dVector(vertices)
        
        if(i%10==0):
            # print(positions)
            #save obj
            _=trimesh.Trimesh(vertices,faces).export(f'meshes/{i}.obj')
            # o3d.io.write_triangle_mesh(f'meshes/{i}.obj', mesh)
            vis.update_geometry(mesh)
            vis.poll_events()
            vis.update_renderer()
    
        vis.capture_screen_image(path_to_save+'/frame'+str(i)+'.png', do_render=True)
        
    vis.destroy_window()
    
        # Save animation as video using ffmpeg, if appropriate
    input_files = path_to_save+ '/frame%d.png'
    output_file = path_to_save+'/output.mp4'
    ffmpeg_path = "/opt/homebrew/bin/ffmpeg"
    os.system(f'{ffmpeg_path} -r 60 -i {input_files} -vcodec libx264 -crf 25 -pix_fmt yuv420p -vf "eq=brightness=0.00:saturation=1.3" {output_file} > /dev/null 2>&1')
    print('Saved video to ' + output_file)
