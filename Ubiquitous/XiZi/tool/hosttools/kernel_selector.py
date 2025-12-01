#! /usr/bin/python3
# this script builds a decision tree to select suitable kernels from developers'  preferences
from __future__ import annotations
from typing import TypeVar, Generic, Tuple, Optional
import os
import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader

ubiquitous_dir = os.environ.get('UBIQUITOUS_ROOT')
kernel_dir = os.environ.get('KERNEL_ROOT')

YAMLS_PATH = f"{ubiquitous_dir}/feature_yaml/"
REQUIREMENT_PATH = f"{kernel_dir}/requirement.yaml"

COMPONENT_KEY = "Feature"
PARAMETER_KEY = "Parameter"
MANUAL_KEY = "Kernel"

# tree node for kernel features
V = TypeVar("V")
class TreeNode(Generic[V]):
    def __init__(self, depth: int, name: str, value: V):
        self.depth = depth
        self.name = name
        self.value = value
        self.children = dict()

    def __repr__(self):
        res = f"L{self.depth}: name {self.name} - value {self.value} - {len(self.children)} children\n"
        for c in self.children:
            # recursively build representation
            res += self.children[c].__repr__() 
        return res
        
    def node_number(self):
        if len(self.children) == 0:
            return 1
        number = 1
        for c in self.children:
            number += self.children[c].node_number()
        return number


# this class should be constructed from the yaml file
class Kernel:
    def __init__(self, name, feature_yaml: dict):
        self.name = name
        self.features_root: Optional[TreeNode[bool]] = None
        self.parameters_root: Optional[TreeNode[int]] = None
        self.extract_components(feature_yaml)
        self.extract_parameters(feature_yaml)
        
    def __repr__(self):
        return f"{self.name}\nFeature Tree:\n{self.features_root}Parameter Tree:\n{self.parameters_root}\n"

    def extract_components(self, feature_yaml: dict):
        self.features_root = TreeNode(0, COMPONENT_KEY, True)
        for k,v in feature_yaml[COMPONENT_KEY].items():
            node = TreeNode(1, k, v)
            if k in feature_yaml:
                self.build_sub_tree(2, node, feature_yaml)
            self.features_root.children[k] = node
         
    def build_sub_tree(self, depth, node: TreeNode[bool], feature_yaml: dict):
        if feature_yaml[node.name] is None:
            return
        for k,v in feature_yaml[node.name].items():
            child = TreeNode(depth, k, v)
            if child.name in feature_yaml:
                self.build_sub_tree(depth+1, child, feature_yaml)
            node.children[k] = child

    def extract_parameters(self, feature_yaml: dict):
        self.parameters_root = TreeNode(0, PARAMETER_KEY, 0)
        for k,v in feature_yaml[PARAMETER_KEY].items():
            child = TreeNode(1, k, v)
            self.parameters_root.children[k] = child

    def check(self, requirements_root, root) -> Tuple[bool, int]:
        distance = 0
        q = list()
        q.append((requirements_root, root))
        while len(q) > 0:
            (r_node, s_node) = q.pop()
            for c in r_node.children:
                if c in s_node.children and self.compare(c, s_node.children[c].value, r_node.children[c].value):
                    q.append((r_node.children[c], s_node.children[c]))
                    distance += (r_node.children[c].value - s_node.children[c].value)**2
                else:
                    return False, distance
        return True, distance

    def compare(self, name, v1, v2):
        if name.endswith("GREATER_THAN"):
            return v1 >= v2
        elif name.endswith("LESS_THAN"):        
            return v1 <= v2
        else:
            return v1 == v2

# load the requirements from the yaml file
def load_required_features()->dict:
    with open(REQUIREMENT_PATH, 'r') as f:
        content = f.read()
        feature_yaml = yaml.load(content, Loader=Loader)
        print(feature_yaml)
        return feature_yaml

# load kernel yamls from ubiquitous directory and build Kernel class
def load_kernel_features()->list[Kernel]:
    kernels = {}
    for file_name in os.listdir(YAMLS_PATH):
        kernel_name = file_name.rstrip(".yaml")
        with open(os.path.join(YAMLS_PATH, file_name), 'r') as f:
            content = f.read()
            feature_yaml = yaml.load(content, Loader=Loader)
            kernels[kernel_name] = Kernel(kernel_name, feature_yaml)
    return kernels

# recommend a suitable kernel according to the requirement
def select_kernel(requirement: dict, kernels: dict[Kernel]) -> str:
    selected = None
    selected_node_number = 0
    selected_distance = 0
    requirement = Kernel("requirement", requirement)
    for name, kernel in kernels.items():
        # here should be a tree matching algorithm
        # requirement trees (both features and parameters) 
        # should be subtrees of the recommended kernel
        pass_features, _ = kernel.check(requirement.features_root, kernel.features_root)
        pass_parameters, distance = kernel.check(requirement.parameters_root, kernel.parameters_root)
        print(name, pass_features, pass_parameters, distance)
        if pass_features and pass_parameters:
            if selected is None:
                selected = kernel
                selected_node_number = kernel.features_root.node_number()
                selected_node_number = distance
            else:
                node_number = kernel.features_root.node_number()
                if selected_node_number > node_number or (selected_node_number == node_number and selected_distance > distance):
                    selected = kernel
                    selected_node_number = kernel.features_root.node_number()
                    selected_node_number = distance
    if selected is None:
        return "Cannot find suitable kernel"
    else:
        return selected.name

if __name__ == "__main__":
    requirement = load_required_features()
    if MANUAL_KEY in requirement:
        selected = list(requirement[MANUAL_KEY].keys())[0][7:]
    else:
        kernels = load_kernel_features()
        selected = select_kernel(requirement, kernels)
    print(f"Selected kernel is: {selected}")