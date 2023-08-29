import pickle
import sys


def create_leaves_to_weights_dict(orig_voc_path: str):
    skip_first_line = True
    leaves_to_weights = {}
    with open(orig_voc_path, mode='r') as f:
        for line in f:
            if skip_first_line:
                skip_first_line = False
                continue

            splitted = line.split()
            # if current row is a leaf, save {<leaf>:<weight>} (both are strings)
            if splitted[1] == '1' and splitted[-1] != '0':
                leaf = ' '.join(splitted[2:-1])
                weight = splitted[-1] 
                leaves_to_weights[leaf] = weight

    return leaves_to_weights

def create_leaves_and_weights_file(leaves_to_weights: dict, output_file: str):
    with open(output_file, 'wb') as handle:
        pickle.dump(leaves_to_weights, handle, protocol=pickle.HIGHEST_PROTOCOL)

def create_leaves_file(leaves_to_weights: dict, output_file: str):
    with open(output_file, mode='w+') as f: #927596
        for k in leaves_to_weights.keys():
            f.write(f'{k}\n')


def main(orig_voc_path,output_path):
    leaves_to_weights : dict = create_leaves_to_weights_dict(orig_voc_path)

    print(len(leaves_to_weights))

    # # uncomment to create leaves file
    output_file = output_path+'/leaves.txt'
    create_leaves_file(leaves_to_weights, output_file)

    # # uncomment to create leaves and weights dict file (pickle)
    output_file = output_path+'/leaves_to_weights.pkl'
    create_leaves_and_weights_file(leaves_to_weights, output_file)


if __name__ == "__main__":
    main(sys.argv[1],sys.argv[2])