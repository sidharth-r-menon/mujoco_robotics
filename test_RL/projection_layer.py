import warp as wp
import numpy as np
import matplotlib.pyplot as plt
from itertools import product, combinations

# Initialize Warp (uses your RTX 3090 automatically)
wp.init()

# -----------------------------------------------------------
# 1. DEFINE THE "HEAT MAP" (Signed Distance Field - SDF)
# -----------------------------------------------------------
# This kernel runs on the GPU. It calculates the distance from a point 'p'
# to a box defined by 'box_center' and 'box_half_size'.
@wp.kernel
def box_sdf_kernel(
    positions: wp.array(dtype=wp.vec3),
    box_center: wp.vec3,
    box_half_size: wp.vec3,
    output_sdf: wp.array(dtype=float)
):
    tid = wp.tid()
    p = positions[tid]
    
    # Transform point to box local frame
    local_p = p - box_center
    
    # Calculate distance to the box surface
    # Logic: q = abs(p) - size
    q = wp.vec3(
        wp.abs(local_p[0]) - box_half_size[0],
        wp.abs(local_p[1]) - box_half_size[1],
        wp.abs(local_p[2]) - box_half_size[2]
    )
    
    # SDF calculation (Standard formula for a Box)
    dist = wp.length(wp.max(q, wp.vec3(0.0, 0.0, 0.0))) + wp.min(wp.max(q), 0.0)
    
    # We want negative distance = inside, positive = outside.
    # But for collision LOSS, we want positive value = penetration depth.
    # So we flip the sign: If inside, return positive magnitude.
    output_sdf[tid] = -dist

# -----------------------------------------------------------
# 2. THE PROJECTION LAYER (The "Safety Filter")
# -----------------------------------------------------------
class ProjectionLayer:
    def __init__(self):
        # Define our Obstacle (The Table)
        # A 1m x 1m x 1m box at the origin
        self.box_center = wp.vec3(0.0, 0.0, 0.0)
        self.box_half_size = wp.vec3(0.5, 0.5, 0.5)
        
        # Optimization parameters
        self.learning_rate = 0.1
        self.max_steps = 50

    def project(self, initial_pos_np):
        """
        Takes a 'Proposed Position' (potentially invalid).
        Returns a 'Valid Position' (guaranteed collision-free).
        """
        # Convert to Warp (GPU) Tensor
        pos_wp = wp.array(initial_pos_np, dtype=wp.vec3, requires_grad=True)
        sdf_output = wp.array([0.0], dtype=float, requires_grad=True)
        
        trajectory = [initial_pos_np[0].copy()] # For visualization only
        
        print(f"Start: {initial_pos_np[0]}")

        # --- The Optimization Loop (Gradient Descent) ---
        for i in range(self.max_steps):
            # Reset SDF output for this iteration
            sdf_output.zero_()
            
            tape = wp.Tape()
            
            # Forward Pass: Record operations to calculate gradients
            with tape:
                wp.launch(
                    kernel=box_sdf_kernel,
                    dim=len(pos_wp),
                    inputs=[pos_wp, self.box_center, self.box_half_size, sdf_output]
                )
            
            # Check current violation (penetration depth)
            current_penetration = sdf_output.numpy()[0]
            
            if current_penetration <= 0.001:
                print(f"✅ Valid at step {i}! Penetration: {current_penetration:.5f}")
                break
                
            # Backward Pass: Calculate gradient (Which way is 'out'?)
            tape.backward(sdf_output)
            
            # Update: Move the object OUT of the wall
            # New_Pos = Old_Pos - (Gradient * Step_Size)
            grad_np = pos_wp.grad.numpy()
            current_pos_np = pos_wp.numpy()
            
            # NOTE: We subtract gradient because we minimized the SDF penetration
            new_pos_np = current_pos_np - (grad_np * self.learning_rate)
            
            # Update the Warp array for next iteration
            pos_wp = wp.array(new_pos_np, dtype=wp.vec3, requires_grad=True)
            
            trajectory.append(new_pos_np[0].copy())
            if i % 10 == 0:
                print(f"Step {i}: Penetration = {current_penetration:.4f} | Pos = {current_pos_np[0]}")

        return pos_wp.numpy(), np.array(trajectory)

# -----------------------------------------------------------
# 3. RUN THE EXPERIMENT
# -----------------------------------------------------------
if __name__ == "__main__":
    layer = ProjectionLayer()
    
    # CASE: The AI puts the scanner INSIDE the table (Invalid)
    # Table is from -0.5 to 0.5. Point (0.1, 0.1, 0.1) is deep inside.
    bad_guess = np.array([[0.1, 0.1, 0.1]], dtype=np.float32)
    
    # Run the "Magic"
    valid_pos, path = layer.project(bad_guess)
    
    print(f"\nFinal Result: {valid_pos[0]}")
    
    # -------------------------------------------------------
    # 4. VISUALIZATION (Proof for your Paper)
    # -------------------------------------------------------
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Draw the Table (Wireframe)
    r = [-0.5, 0.5]
    for s, e in combinations(np.array(list(product(r, r, r))), 2):
        if np.sum(np.abs(s-e)) == r[1]-r[0]:
            ax.plot3D(*zip(s, e), color="red", alpha=0.3)
            
    # Draw the "Correction Path"
    path_x = path[:, 0]
    path_y = path[:, 1]
    path_z = path[:, 2]
    
    ax.scatter(path_x[0], path_y[0], path_z[0], c='red', s=100, label='Bad Input (RL)')
    ax.scatter(path_x[-1], path_y[-1], path_z[-1], c='green', s=100, label='Projected Output')
    ax.plot(path_x, path_y, path_z, c='blue', linestyle='--', label='Gradient Projection')
    
    ax.set_title("Differentiable Constraint Projection (SDF Gradient Descent)")
    ax.legend()
    plt.show()