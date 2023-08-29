import complete_weights
import argparse
import subprocess
import os
import pickle
import numpy as np



# NOTE: replace this with pickle of weights

def load_leaves_to_weights(leaves_to_weights_path):
    leaves_to_weights = None
    with open(leaves_to_weights_path, 'rb') as handle:
        leaves_to_weights = pickle.load(handle)
    assert leaves_to_weights is not None, f'failed to load {leaves_to_weights_path}'
    return leaves_to_weights

def update_weights(temp_tree_file: str, final_tree_file: str, model_fn, predict_fn,leaves_to_weights_path):
    leaves_count = 0
    unweighted_leaves_count = 0
    leaves_to_weights: dict = load_leaves_to_weights(leaves_to_weights_path)
    weights = list(leaves_to_weights.values())
    weights = [float(w) for w in weights]
    model = model_fn(leaves_to_weights_path) if model_fn is not None else None
    with open(temp_tree_file, mode='r') as temp_tree:
        with open(final_tree_file, mode='w+') as final_tree:
            first_line = True
            for line in temp_tree:
                if first_line:
                    final_tree.write(f'{line}')
                    first_line = False
                    continue

                splitted = line.split()
                # if current node is not a leaf, write as is
                if splitted[1] != '1':
                    final_tree.write(f'{line}')
                # if current node is a leaf, write it with it's corresponding weight to final tree
                else:
                    leaves_count += 1
                    leaf = ' '.join(splitted[2:-1])
                    #leaf_weight = leaves_to_weights.get(leaf, str(quarter_precentile))
                    leaf_weight = leaves_to_weights.get(leaf, '0')
                    if leaf_weight == '0':
                        unweighted_leaves_count += 1
                        if model_fn is not None:
                            leaf_weight = str(predict_fn(model, leaf))
                    updated_line = splitted[:-1]
                    updated_line.append(leaf_weight)
                    updated_line = ' '.join(updated_line)
                    final_tree.write(f'{updated_line}\n')
        
    print(f'created tree has {leaves_count} overall leaves, {unweighted_leaves_count} unweighted.')
    # remove temproray tree file
    os.remove(temp_tree_file)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--leaves_path', help="path to leaves file", required=True)
    parser.add_argument('-b', '--branching_factor', help="number of clusters on each level", type=int, required=True)
    parser.add_argument('-d', '--depth', help="maximal tree depth (for malloc)", type=int, required=True)
    parser.add_argument('-e', '--effective_depth', help="maximal tree depth (effective)", type=int, required=True)
    parser.add_argument('-o', '--output_file', help="file name to dump newly created tree file (in current directrory)", required=True)
    parser.add_argument('-s', '--script_path', help="location of the create_tree.cpp exe after build", required=True)
    parser.add_argument('-w', '--weighting_method', help="weighting method ['gaussian', 'knn', 'dl']", default=None)
    parser.add_argument('-lw', '--leaves_to_weights_path', help="path to leaves_to_weights_path.pkl file", default=None)
    args = parser.parse_args()
    create_trees_script = args.script_path
    leaves_path = args.leaves_path
    leaves_to_weights_path = args.leaves_to_weights_path
    branching_factor = str(args.branching_factor)
    depth = str(args.depth)
    effective_depth = str(args.effective_depth)
    output_file = args.output_file
    temp_output_file = f'temp_tree.txt'
    weighting_method = args.weighting_method

    create_tree_cmd = [create_trees_script, leaves_path, branching_factor, depth, effective_depth, temp_output_file]
    print(f'running: {" ".join(create_tree_cmd)}')
    # run c++ 'create_trees', this creates the tree in 'temp_<output_file>', but without proper weights
    ret_code = subprocess.call(args=create_tree_cmd)
    assert ret_code==0, 'failed to create vocabulary tree, halting'
    # overwrite 'output_file' with fixed weights from dict
    print(f'updating tree weights and saving to {output_file}')
    if weighting_method == 'knn':
        model_fn = complete_weights.prepare_knn_model
        predict_fn = complete_weights.predict_knn_regressor
    elif weighting_method == 'gaussian':
        model_fn = complete_weights.get_normal_ditribution
        predict_fn = complete_weights.sample_from_distribution
    elif weighting_method == 'dl':
        model_fn = complete_weights.get_model_for_predict
        predict_fn = complete_weights.predict_with_linear_model
    else:
        model_fn = None
        predict_fn = None
    
    update_weights(temp_output_file, output_file, model_fn, predict_fn,leaves_to_weights_path)


if __name__ == '__main__':
    main()