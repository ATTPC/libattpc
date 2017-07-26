# libattpc

A cross-platform analysis library for the AT-TPC

## Cloning the repository

Cloning this repository requires a few extra steps since it includes Git submodules. To clone a copy
of this repository onto your computer, use the command

    git clone --recursive https://github.com/attpc/libattpc.git

The main difference from the usual process is the extra `--recursive` flag.

Alternatively, if you already have a copy and the submodules in the `libs` directory are empty, run the command

    git submodule update --init --recursive

to check out the appropriate version of each submodule.

For more information about working with submodules, see the [Git book](https://git-scm.com/book/en/v2/Git-Tools-Submodules).
