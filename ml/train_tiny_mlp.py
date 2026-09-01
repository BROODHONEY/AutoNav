import numpy as np

np.random.seed(42)

N = 4000
distance = np.random.uniform(5, 30, N)
speed    = np.random.uniform(1, 4, N)
time     = distance / speed + np.random.normal(0, 0.15, N)

x1 = distance / 30.0
x2 = speed / 4.0
y  = time / 30.0

X = np.stack([x1, x2], axis=1)
Y = y.reshape(-1, 1)

rng = np.random.default_rng(0)
H = 8  # hidden units
W1 = rng.normal(0, 0.4, (2,H))
b1 = np.zeros((1,H))
W2 = rng.normal(0, 0.4, (H,1))
b2 = np.zeros((1,1))

lr = 0.08
epochs = 12000

for e in range(epochs):
    Z1 = X @ W1 + b1
    A1 = np.maximum(0, Z1)
    Z2 = A1 @ W2 + b2
    loss = np.mean((Z2 - Y)**2)

    dZ2 = 2*(Z2 - Y) / N
    dW2 = A1.T @ dZ2
    db2 = dZ2.sum(axis=0, keepdims=True)
    dA1 = dZ2 @ W2.T
    dZ1 = dA1 * (Z1 > 0)
    dW1 = X.T @ dZ1
    db1 = dZ1.sum(axis=0, keepdims=True)

    W1 -= lr*dW1; b1 -= lr*db1
    W2 -= lr*dW2; b2 -= lr*db2

    if e % 2000 == 0:
        print(f"epoch {e}, loss {loss:.5f}")

print(f"final loss: {loss:.5f}")

def fmt(arr, name):
    flat = arr.flatten()
    vals = ", ".join(f"{v:.6f}f" for v in flat)
    print(f"const float {name}[{len(flat)}] = {{{vals}}};")

print()
print("// ---- copy these into the Arduino sketch ----")
fmt(W1, "W1")
fmt(b1, "b1")
fmt(W2, "W2")
fmt(b2, "b2")

print()
print("// sanity checks")
for d, s in [(10,2), (20,1.5), (5,4), (25,3), (8,1)]:
    xi1, xi2 = d/30.0, s/4.0
    z1 = np.array([xi1, xi2]) @ W1 + b1
    a1 = np.maximum(0, z1)
    z2 = a1 @ W2 + b2
    pred_time = max(0.0, float(z2[0,0]) * 30.0)
    print(f"distance={d}, speed={s} -> predicted time={pred_time:.2f} (true={d/s:.2f})")
