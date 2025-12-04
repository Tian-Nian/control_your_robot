'''
实现多个子任务连续执行
'''
import abc
from typing import List, Optional, Any, Iterable, Union
import os
import yaml
import importlib

from utils.data_handler import debug_print

class BaseTask(abc.ABC):
    @abc.abstractmethod
    def is_success(self) -> bool:
        pass
    
    @abc.abstractmethod
    def run(self):
        pass

    @abc.abstractmethod
    def is_fail(self) -> bool:
        pass

    def update(self) -> None:
        pass

class SerialTask(BaseTask):
    def __init__(self, subtasks: List[BaseTask]):

        self.subtasks = subtasks
        self.current_idx = 0
    
    def run(self):
        current = self.subtasks[self.current_idx]
        current.run()
    def update(self) -> None:
        current = self.subtasks[self.current_idx]
        current.update()
        if current.is_success() and self.current_idx < len(self.subtasks) - 1:
            self.current_idx += 1

        if current.is_fail():
            self.current_idx -= 1
    def is_success(self) -> bool:
        return self.current_idx == len(self.subtasks) - 1 and self.subtasks[-1].is_success()

    def is_fail(self) -> bool:
        if self.current_idx >= 0:
            return self.subtasks[self.current_idx].is_fail()
        else:
            return True

class ParallelTask(BaseTask):
    def __init__(self, subtasks: List[BaseTask]):
        self.subtasks = subtasks

    def run(self):
        for t in self.subtasks:
            t.run()
    def is_fail(self):
        return any(t.is_fail() for t in self.subtasks)
    def is_success(self) -> bool:
        return all(t.is_success() for t in self.subtasks)
    def update(self) -> None:
        for t in self.subtasks:
            t.update()

class Success(BaseTask):
    def is_success(self):
        return True
    def is_fail(self):
        return False
    def run(self):
        return

class Tasks:
    @staticmethod
    def build(config: Optional[dict[str, Any] | BaseTask]) -> BaseTask:
        if isinstance(config, BaseTask):
            return config

        if not isinstance(config, dict):
            raise ValueError(f"Invalid config type: {type(config)}")

        task_type = config.get("type", "Serial")

        if task_type == "Serial":
            subtasks = [Tasks.build(s) for s in config["subtasks"]]
            return SerialTask(subtasks=subtasks)

        elif task_type == "Parallel":
            subtasks = [Tasks.build(s) for s in config["subtasks"]]
            return ParallelTask(subtasks=subtasks)
        else:
            # 默认就是叶子 SubTask
            return config

    @staticmethod
    def build_top(config: dict[str, Any]) -> BaseTask:
        """Build top-level task with enforced normalization."""
        task = Tasks.build(config)

        class TopLevelTaskWrapper(BaseTask):
            def run(self) -> None:
                task.run()

            def update(self) -> None:
                task.update()

            def is_success(self) -> bool:
                return task.is_success()

            def is_fail(self) -> bool:
                return task.is_fail()

        return TopLevelTaskWrapper()

"""
使用yml初始化任务, 包括以下参数:

"""
def get_class(import_name, class_name):
    try:
        class_module = importlib.import_module(import_name)
        debug_print("function", f"Module loaded: {class_module}", "DEBUG")
    except ModuleNotFoundError as e:
        raise SystemExit(f"ModuleNotFoundError: {e}")

    try:
        return_class = getattr(class_module, class_name)
        debug_print("function", f"Class found: {return_class}", "DEBUG")

    except AttributeError as e:
        raise SystemExit(f"AttributeError: {e}")
    except Exception as e:
        raise SystemExit(f"Unexpected error instantiating model: {e}")
    return return_class

'''
要给予yml文件配置任务, 需要拥有以下信息:
name:
  - task_name
robot:
  - class:
    - class_path:
      - path
    - class_name:
      - name
  - args:
    - ...
run:
  - function:
     - function_path:
        - path
      - function_name:
        - name
  - args:
    - ...
  ...
success:
    - function:
        - function_path:
            - path
        - function_name:
            - name
  - args:
    - ...
# 可选:
extra:
  - extra_1:
    - class:
      - class_path
      - class_name
    - args:
      - ...


fail:
    - function:
        - function_path:
            - path
        - function_name:
            - name
  - args:
    - ...
  - release:
    True / False
'''
class YmlTask(BaseTask):
    def __init__(self, yml_path: str, is_block=True):
        self.is_block = is_block
        self.running = False
        self.args = None
        self.success = False

        self.robot = None
        self.run_once = None
        self.extras = None
        self.extra_classes = None

        if os.path.exists(yml_path):
            with open(yml_path, "r", encoding="utf-8") as f:
                self.args = yaml.safe_load(f)
        if self.args is None:
            raise ValueError(f"Invalid yml file: {yml_path}")
        
        try:
            self.name = self.args["name"]
        except:
            raise ValueError(f"yml file must have name")

        try:
            self.run_func = get_class(self.args["run"]["function"]["function_path"], self.args["run"]["function"]["function_name"])
        except:
            raise ValueError(f"yml file must have run")
        
        try:
            self.success_func = get_class(self.args["success"]["function"]["function_path"], self.args["success"]["function"]["function_name"])
        except:
            raise ValueError(f"yml file must have success")
        
        try:
            self.robot_class = get_class(self.args["robot"]["class"]["class_path"], self.args["robot"]["class"]["class_name"])
        except:
            raise ValueError(f"yml file must have robot")
        
        if self.args.get("fail") is not None:
            self.fail_func = get_class(self.args["fail"]["function"]["function_path"], self.args["fail"]["function"]["function_name"])
        else:
            debug_print(self.name, f"Fail function not found", "INFO")
        
        if self.args.get("extras") is not None:
            self.extra_classes = {}
            extra_names = self.args["extras"].keys()
            for extra_name in extra_names:
                if extra_name == "release":
                    continue
                self.extra_classes[extra_name] = get_class(self.args["extras"][extra_name]["class"]["class_path"], self.args["extras"][extra_name]["class"]["class_name"])
        

    def run(self):
        self.success = False

        if self.robot is None:
            if self.args["robot"]["args"] is not None:
                self.robot = self.robot_class(**self.args["robot"]["args"])
            else:
                self.robot = self.robot_class()
            self.robot.set_up()
        if self.extra_classes is not None and self.extras is None:
            self.extras = {}
            for extra_name, extra_class in self.extra_classes.items():
                if self.args["extras"][extra_name]["args"] is not None:
                    self.extras[extra_name] = extra_class(**self.args["extras"][extra_name]["args"])
                else:
                    self.extras[extra_name] = extra_class()
        
        if self.is_block:
            if self.running:
                return
            
        if self.run_once is None:
            if self.args["run"]["args"] is not None:
                self.run_once = self.run_func(self, **self.args["run"]["args"])
            else:
                self.run_once = self.run_func(self)

        self.running = True
    
    def is_success(self):
        if self.success:
            return True
        
        if self.robot is None:
            if self.args["robot"]["args"] is not None:
                self.robot = self.robot_class(**self.args["robot"]["args"])
            else:
                self.robot = self.robot_class()
            self.robot.set_up()
        if self.extra_classes is not None and self.extras is None:
            self.extras = {}
            for extra_name, extra_class in self.extra_classes.items():
                if self.args["extras"][extra_name]["args"] is not None:
                    self.extras[extra_name] = extra_class(**self.args["extras"][extra_name]["args"])
                else:
                    self.extras[extra_name] = extra_class()
        
        if self.args["success"]["args"] is not None:
            success = self.success_func(self, **self.args["success"]["args"])
        else:
            success = self.success_func(self)
        
        if success:
            debug_print(self.name, "success!", "INFO")
            self.success = True
            self.robot = None
            self.running = False
            if self.extras is not None:
                if self.args["extras"]["release"]:
                    self.extras = None
        
        return success

    def is_fail(self):
        if self.success:
            return False

        if self.args.get("fail") is not None:
            if self.robot is None:
                if self.args["robot"]["args"] is not None:
                    self.robot = self.robot_class(**self.args["robot"]["args"])
                else:
                    self.robot = self.robot_class()
                self.robot.set_up()
            if self.extra_classes is not None and self.extras is None:
                self.extras = {}
                for extra_name, extra_class in self.extra_classes.items():
                    if self.args["extras"][extra_name]["args"] is not None:
                        self.extras[extra_name] = extra_class(**self.args["extras"][extra_name]["args"])
                    else:
                        self.extras[extra_name] = extra_class()
            
            if self.args["fail"]["args"] is not None:
                fail = self.fail_func(self, **self.args["fail"]["args"])
            else:
                fail = self.fail_func(self)
            
            if fail:
                debug_print(self.name, "fail!", "INFO")
                self.success = False
                self.robot = None
                self.running = False
                if self.extras is not None:
                    if self.args["extras"]["release"]:
                        self.extras = None
                
            return fail
        else:
            return False