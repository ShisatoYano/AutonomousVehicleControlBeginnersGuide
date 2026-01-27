import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor
from sklearn.linear_model import LinearRegression

# =====================================================
# Fit a line ax + by + c = 0 using least squares
# =====================================================
def fit_line(points):
    X = points[:, 0][:, None]
    y = points[:, 1]

    ransac = RANSACRegressor(LinearRegression(), residual_threshold=0.03)
    ransac.fit(X, y)

    a = -ransac.estimator_.coef_[0]
    b = 1.0
    c = -ransac.estimator_.intercept_

    norm = np.sqrt(a*a + b*b)
    return np.array([a, b, c]) / norm, ransac.inlier_mask_


# =====================================================
# Line intersection
# =====================================================
def intersect_lines(l1, l2):
    a1, b1, c1 = l1
    a2, b2, c2 = l2

    A = np.array([[a1, b1],
                  [a2, b2]])
    b = -np.array([c1, c2])

    return np.linalg.solve(A, b)


# =====================================================
# MAIN
# =====================================================
if __name__ == "__main__":
    np.random.seed(0)

    # ----- synthetic 2-face LiDAR data -----
    corner = np.array([1.0, 2.0])
    phi = np.deg2rad(90)

    c, s = np.cos(phi), np.sin(phi)
    t = np.linspace(0, 1.5, 60)

    face1 = np.column_stack([corner[0] + t*c,
                             corner[1] + t*s])
    face2 = np.column_stack([corner[0] - t*s,
                             corner[1] + t*c])

    points = np.vstack([face1, face2])
    points += 0.02 * np.random.randn(*points.shape)

    # ----- split faces using PCA projection -----
    mean = points.mean(axis=0)
    U, S, Vt = np.linalg.svd(points - mean)
    proj = (points - mean) @ Vt[0]

    cluster1 = points[proj > 0]
    cluster2 = points[proj <= 0]

    # ----- fit lines -----
    line1, in1 = fit_line(cluster1)
    line2, in2 = fit_line(cluster2)

    # ----- intersect -----
    corner_est = intersect_lines(line1, line2)

    print("Estimated corner:", corner_est)

    # ----- plot -----
    x = np.linspace(points[:,0].min()-0.5, points[:,0].max()+0.5, 100)

    y1 = (-line1[0]*x - line1[2]) / line1[1]
    y2 = (-line2[0]*x - line2[2]) / line2[1]

    plt.figure(figsize=(6,6))
    plt.scatter(points[:,0], points[:,1], s=10, alpha=0.6)
    plt.plot(x, y1, 'r-', lw=2, label='Face 1')
    plt.plot(x, y2, 'g-', lw=2, label='Face 2')
    plt.plot(corner_est[0], corner_est[1], 'ko', label='Corner')

    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title("2D LiDAR Corner Detection (Correct Way)")
    plt.show()
