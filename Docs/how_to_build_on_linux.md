<h1>How to build CARLA on Linux</h1>

!!! note
    CARLA requires Ubuntu 16.04 or later.

Install the build tools and dependencies

    $ sudo apt-get install build-essential clang-3.9 git cmake ninja-build python3-requests python-dev tzdata sed curl wget unzip autoconf libtool

To avoid compatibility issues between Unreal Engine and the CARLA dependencies,
the best configuration is to compile everything with the same compiler version
and C++ runtime library. We use clang 3.9 and LLVM's libc++. You may need to
change your default clang version to compile Unreal

    $ sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-3.9/bin/clang++ 100
    $ sudo update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-3.9/bin/clang 100

Build Unreal Engine
-------------------

!!! note
    Unreal Engine repositories are set to private. In order to gain access you
    need to add your GitHub username when you sign up at
    [www.unrealengine.com](https://www.unrealengine.com).

Download and compile Unreal Engine 4.18. Here we will assume you install it at
"~/UnrealEngine_4.18", but you can install it anywhere, just replace the path
where necessary.

    $ git clone --depth=1 -b 4.18 https://github.com/EpicGames/UnrealEngine.git ~/UnrealEngine_4.18
    $ cd ~/UnrealEngine_4.18
    $ ./Setup.sh && ./GenerateProjectFiles.sh && make

Check Unreal's documentation
["Building On Linux"](https://wiki.unrealengine.com/Building_On_Linux) if any of
the steps above fail.

Build CARLA
-----------

Clone or download the project from our
[GitHub repository](https://github.com/carla-simulator/carla)

    $ git clone https://github.com/carla-simulator/carla

Note that the `master` branch contains the latest fixes and features, for the
latest stable code may be best to switch to the `stable` branch.

Run the setup script to download the content and build all dependencies. It
takes a while

    $ ./Setup.sh

Once it's done it should print "Success" if everything went well.

To build CARLA, use the rebuild script. This script deletes all intermediate
files, rebuilds whole CARLA, and launches the editor. Use it too for making a
clean rebuild of CARLA

    $ UE4_ROOT=~/UnrealEngine_4.18 ./Rebuild.sh

It looks at the environment variable `UE4_ROOT` to find the right version of
Unreal Engine. You can also add this variable to your "~/.bashrc" or similar.

Later, if you need to compile some changes without doing a full rebuild, you can
use the Makefile generated in the Unreal project folder

    $ cd Unreal/CarlaUE4
    $ make CarlaUE4Editor

Updating CARLA
--------------

Every new release of CARLA we release a new package with the latest changes in
the CARLA assets. To download the latest version, run the "Update" script

    $ git pull
    $ ./Update.sh

Launching the editor
--------------------

To open the editor once the project is already built

    $ cd Unreal/CarlaUE4
    $ ~/UnrealEngine_4.18/Engine/Binaries/Linux/UE4Editor "$PWD/CarlaUE4.uproject"

Test (Optional)
---------------

A set of unit tests is available for testing the CarlaServer library (note that
these tests launch the python client, they require python3 and protobuf for
python3 installed, as well as ports 2000 and 4000 available)

    $ make check
