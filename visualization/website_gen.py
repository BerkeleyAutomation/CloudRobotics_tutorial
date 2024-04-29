import fog_x

DATASETS = [
    "demo_ds"
]

for dataset_name in DATASETS:
    dataset = fog_x.dataset.Dataset(
        name=dataset_name,
        path="s3://cloud-robotics-workshop",
    )
    