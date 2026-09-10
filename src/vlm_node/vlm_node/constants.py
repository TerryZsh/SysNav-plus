import os

# ---- Domestic DashScope/Qwen only ----
VLM_PROVIDER = "qwen"
VLM_API_KEY = os.environ.get("DASHSCOPE_API_KEY", "")
VLM_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
MODEL_NAME = os.environ.get("QWEN_MODEL", "qwen3.6-plus")
MODEL_NAME_LITE = os.environ.get("QWEN_MODEL_LITE", "qwen3.6-flash")

# Target object to search for in the scene
target_object = ""
room_condition     = ""
spatial_condition  = ""
anchor_object      = ""
attribute_condition = ""
