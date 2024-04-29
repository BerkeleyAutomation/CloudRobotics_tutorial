#!/bin/bash

set -e

if [[ -z "$USER" || -z "$HOME" || -z "$WORKDIR" ]] ; then
    echo "One or more required environment variables is not defined."
    echo "USER: $USER"
    echo "HOME: $HOME"
    echo "WORKDIR: $WORKDIR"
    exit 1
fi

if [[ -z "$USER_ID" ]] ; then
    # Windows hosts do not have the same permission issues, we can
    # skip USER_ID and GROUP_ID if not provided.
    echo "Starting with USER: $USER"
    useradd -s /bin/bash -d "$HOME" -G sudo -c "local user" -M "$USER"
    USER_ID=`id -u $USER`
    GROUP_ID=`id -g $USER`
else
    # MacOS and Linux hosts will write files with the user and group
    # ids of the docker VM.  This usually results in files that are
    # uneditable without a sudo chmod on the host.  Here we create a
    # user with matching user and group ids.
    echo "Starting with USER: $USER, UID: $USER_ID, GID: $GROUP_ID"
    # Check if group exists, create if not
    if ! getent group $GROUP_ID > /dev/null; then
        echo "Creating group with GID: $GROUP_ID"
        groupadd -g $GROUP_ID group$GROUP_ID
    fi
    useradd -s /bin/bash -d "$HOME" -u "$USER_ID" -g "$GROUP_ID" -G sudo -N -o -c "local user" -M "$USER"
fi

# Fix up permissions for all parent directories of WORKDIR so that
# they're owned by the user we just created.  All directories between
# / or $HOME and $WORKDIR will be created automatically, but with the
# wrong user.
dir="$WORKDIR"
while [[ "$dir" != "/" && "$dir" != "$HOME" ]] ; do
    chown "$USER_ID:$GROUP_ID" "$dir"
    dir="$(dirname -- "$dir")"
done

# On the chance that $WORKDIR is not in $HOME, we need to create $HOME
mkdir -p "$HOME"

chown "$USER_ID:$GROUP_ID" "$HOME"

gosu "$USER_ID:$GROUP_ID" cp -rT /etc/skel "$HOME"

# setup ros2 environment
# echo 'source "/opt/ros/$ROS_DISTRO/setup.bash" --' >> "$HOME/.bashrc"
echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> "$HOME/.bashrc"
echo "source \"/fog_ws/install/setup.bash\"" >> "$HOME/.bashrc"
exec gosu "$USER_ID:$GROUP_ID" "$@"
