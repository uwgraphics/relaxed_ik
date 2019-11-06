import sys, string, os


def get_all_files_in_dir(fp):
    files = []
    # r=root, d=directories, f = files
    for rr, d, f in os.walk(fp):
        for file in f:
            files.append(file)

    return files

def is_file_in_dir(fp, file_name):
    files = get_all_files_in_dir(fp)
    return file_name in files