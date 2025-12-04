python example/deploy/deploy.py \
    --base_model_name "test_policy"\
    --base_model_class "TestModel"\
    --base_model_path "path/to/ckpt"\
    --base_task_name "test"\
    --base_robot_name "test_robot"\
    --base_robot_class "TestRobot"\
    --robotwin \
    --overrides \
    --test_info_1 1
    # --video "cam_head"\