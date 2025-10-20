# See documentation in
# https://docs.prophesee.ai/stable/data/file_formats/hdf5.html#example-using-h5py
import os
import h5py
import argparse


def create_arg_parser():
    parser = argparse.ArgumentParser(
                    prog="inspect_hdf5",
                    description="This script inspect an hdf5 file produced \
                    by an Event Camera from Prophesee",
                    epilog="NutMade")
    parser.add_argument("-i", type=str,
                        help=".hdf5 file to analize", required=True)
    if os.path.exists("./custom_filter.path"):
        with open("custom_filter.path", "r") as custom_filter_file:
            os.environ['HDF5_PLUGIN_PATH'] = custom_filter_file.read()
    else:
        print("The custom_filter.path file does not exist.")
        print("Install the Prophesee filter \
            from https://github.com/prophesee-ai/hdf5")
        custom_filter_file_content = \
            input("Give me the build folder absolute path:")
        with open("custom_filter.path", "w") as custom_filter_file:
            custom_filter_file.write(custom_filter_file_content)
        os.environ['HDF5_PLUGIN_PATH'] = custom_filter_file_content
    return parser


def main():
    parser = create_arg_parser()
    args = parser.parse_args()
    with h5py.File(args.i, "r") as f:
        datasets_names = []

        def add_dataset(name):
            if not isinstance(f[name], h5py.Group):
                datasets_names.append(name)

        f.visit(add_dataset)

        if len(f.attrs) > 0:
            print("DATASET ATTRIBUTES")
            for attr_name in f.attrs:
                print(f"\t{attr_name}: {f.attrs[attr_name]}")
        print()
        print()

        for dataset_name in datasets_names:
            dataset = f[dataset_name]

            print(f"Analyzing {dataset_name}")
            print(f"\tDataset shape: {dataset.shape}")
            if len(f[dataset_name].attrs) > 0:
                print("\tATTRIBUTES")
                for attr_name in f[dataset_name].attrs:
                    print(f"\t\t{attr_name}: \
                              {f[dataset_name].attrs[attr_name]}")
            print("\tFIELDS:")
            for key_name in dataset.dtype.fields:
                print(f"\t\t{key_name} \
                      [{dataset.dtype.fields[key_name][0].name}]")

            if dataset.shape[0] > 5:
                print("\tExample:")
                for i in range(5):
                    print(f"\t\t[{i}]  ", end="")
                    for key_name in dataset.dtype.fields:
                        print(f"{key_name}: {dataset[i][key_name]}", end=" ")
                    print()
                print("\t\t...")
                for i in range(-5, 0):
                    print(f"\t\t[{dataset.shape[0] + i}]  ", end="")
                    for key_name in dataset.dtype.fields:
                        print(f"{key_name}: {dataset[i][key_name]}", end=" ")
                    print()
            else:
                print("\tNot enough data in the dataset for showing examples!")
            print()


if __name__ == "__main__":
    main()
