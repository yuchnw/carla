import numpy as np

W, H = 3840, 2160
f = 4415.697690693613          # pick fx (match HFOV)
max_angle = 0.5             # ~corner angle for this f

# Fit r(theta) = f * tan(theta)
theta = np.linspace(0, max_angle, 2000)
r = f * np.tan(theta)

# polynomial form: r ≈ c0 + c1*θ + c2*θ^2 + ... + c5*θ^5
A = np.vstack([theta**k for k in range(6)]).T
c_angle_to_r, *_ = np.linalg.lstsq(A, r, rcond=None)

# Now fit theta(r) = arctan(r/f)
rmax = r[-1]
rr = np.linspace(0, rmax, 2000)
th = np.arctan(rr / f)

B = np.vstack([rr**k for k in range(6)]).T
c_r_to_angle, *_ = np.linalg.lstsq(B, th, rcond=None)

print("angle_to_pixeldist_poly =", c_angle_to_r.tolist())
print("pixeldist_to_angle_poly =", c_r_to_angle.tolist())