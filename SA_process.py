import csv
import pickle

path = "salib_morris_multi_metric_20260203_064654.pkl"
output_csv = "morris_full_table.csv"

with open(path, "rb") as f:
    data = pickle.load(f)

metric_names = data["metric_names"]
param_names = data["problem"]["names"]

rows = []
for metric in metric_names:
    Si = data["Si_dict"][metric]
    mu = Si.get("mu", [])
    mu_star = Si.get("mu_star", [])
    sigma = Si.get("sigma", [])
    mu_star_conf = Si.get("mu_star_conf", [])

    for i, param_name in enumerate(param_names):
        rows.append({
            "metric": metric,
            "parameter": param_name,
            "mu": mu[i] if i < len(mu) else None,
            "mu_star": mu_star[i] if i < len(mu_star) else None,
            "sigma": sigma[i] if i < len(sigma) else None,
            "mu_star_conf": mu_star_conf[i] if i < len(mu_star_conf) else None,
        })

with open(output_csv, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=["metric", "parameter", "mu", "mu_star", "sigma", "mu_star_conf"])
    writer.writeheader()
    writer.writerows(rows)

print(f"Wrote full Morris table to {output_csv} ({len(rows)} rows)")
