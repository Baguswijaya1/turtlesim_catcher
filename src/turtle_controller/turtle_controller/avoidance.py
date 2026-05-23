import numpy as np

def sense(x1, y1, x2, y2, heading):
    dx = x2-x1
    dy = y2-y1
    distance = np.sqrt(dx**2 + dy**2)

    theta = np.arctan2(dy, dx) - heading
    theta = np.arctan2(np.sin(theta), np.cos(theta)) # normalize to range -pi to pi
    
    return distance, theta

def get_avoidance_vector(x, y, obsX, obsY, heading, avoid_radius, repulsive_gain):
    obs_dist, obs_angle = sense(obsX, obsY, x, y, heading)
    if obs_dist <= avoid_radius:
        f_repuslive = repulsive_gain * (1./(obs_dist+1e-3) - 1./avoid_radius) * 1./(obs_dist+1e-3)**2
        angle_repulsive = obs_angle
        return f_repuslive, angle_repulsive, obs_dist

    return 0.0, 0.0, obs_dist