import os

# Place this in the preamble of your Latex project
# \usepackage{dirtree}

# E.g. C:/Users/user/Desktop/Folder/ on windows or /root/folder/ on linux.
dir = 'C:\Users\alber\OneDrive - Aalborg Universitet\GitHub\KF_IMUandGNSS'

# Link to the Github e.g. https://github.com/UnknownDK/ES7-Quantum/
gitlink = "https://github.com/AlbertHansen/KF_IMUandGNSS"

# Ignore list. This can be specific files/folders or even file extensions e.g. script.py, secretfolder, .git.
ignore = [".git", ".csv"]

def list_files(startpath):
    print("\dirtree{%")
    for root, dirs, files in os.walk(startpath):
        if not any(ele in root for ele in ignore):
            level = root.replace(startpath, '').count(os.sep) + 1
            if os.path.basename(root) != "":
                level += 1
            folders = root.replace('\\', '/').split('/')
            for fold in reversed(folders):
                if fold in dir:
                    del(folders[folders.index(fold)])
            folderpath = ""
            for folder in folders:
                folderpath += folder + "/"
            print('.' + str(level) + ' \href{' + gitlink + 'tree/master/' + folderpath + '}{' + os.path.basename(root).replace("_", "\_") + '/}.' )
            for f in files:
                if not any(ele in f for ele in ignore):
                    filename = f.replace("_", "\_")
                    print('.' + str(level + 1) + ' \href{' + gitlink + 'blob/master/' + folderpath + f + '}{' + filename + '}.' )
    print("}")

list_files(dir)
