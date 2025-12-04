from huggingface_hub import snapshot_download

snapshot_download(
    repo_id="WadeKe/drAloha",
    allow_patterns=["dr.zip"],
    local_dir=".",
    repo_type="dataset",
    resume_download=True,
    local_dir_use_symlinks="auto"
)
