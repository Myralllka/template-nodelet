#!/bin/bash

# set -o xtrace
set -o errexit
set -o nounset
set -o pipefail

PROJECT_NAME=example_project
NAMESPACE_NAME=example_namespace_name
CLASS_NAME=example_class_name
AUTHOR_EMAIL=example_email
AUTHOR_NAME=example_name
PROJECT_DESCRIPTION=example_project_description

CMAKE_PACKAGES=()


while [ $# -ge 1 ]; do
  case $1 in
  -pn | --project-name)
    PROJECT_NAME=$2
    shift 2
    ;;
  -nn | --namespace-name)
    NAMESPACE_NAME=$2
    shift 2
    ;;
  -cn | --class-name)
    CLASS_NAME=$2
    shift 2
    ;;
  -an | --author-name)
    AUTHOR_NAME=$2
    shift 2
    ;;
  -ae | --author-email)
    AUTHOR_EMAIL=$2
    shift 2
    ;;
  -pd | --project-description)
    PROJECT_DESCRIPTION=$2
    shift 2
    ;;
 -cp | --cmake-package)
    CMAKE_PACKAGES+=($2)
    shift 2
    ;;
  -h | --help)
    echo "
  Usage: ./mrs_create_nodelet_template.sh [options]
  Options:
    -h      --help                  Show help message.
    -pn     --project-name          Project name. example_project by default.
    -pd     --project-description  
    -cp     --cmake-packages        Used for cmake "find_package". roscpp and mrs_lib are default packages. Write packages as a one space-seperated string (in columns). Example: 'std_msgs roscpp mrs_lib'
    -nn     --namespace-name        Namespace
    -cn     --class-name            Nodelet class name
    -an     --author-name           Author name
    -ae     --author-email          Should be cpecified for correct compilation!
    "
    exit 0
    ;;
  \?)
    echo "Invalid option: -$OPTARG" >&2
    exit 1
    ;;
  :)
    echo "Option -$OPTARG requires an numerical argument." >&2
    exit 1
    ;;
  *)
    break
    ;;
  esac
done

printf -v CMAKE_PACKAGES "%s " "${CMAKE_PACKAGES[@]}"

CMAKE_PACKAGES=${CMAKE_PACKAGES%?}

sed -i "s/more_project_name/$PROJECT_NAME/g" ./CMakeLists.txt || exit 1
sed -i "s/more_filesname/$CLASS_NAME/g" ./CMakeLists.txt || exit 1
sed -i "s/more_cmake_packages/$CMAKE_PACKAGES/g" ./CMakeLists.txt || exit 1
echo -e "CMakeLists.txt done"

sed -i "s/more_project_name/$PROJECT_NAME/g" ./plugins.xml || exit 1
sed -i "s/more_class_name/$CLASS_NAME/g" ./plugins.xml || exit 1
sed -i "s/more_namespace_name/$NAMESPACE_NAME/g" ./plugins.xml || exit 1
echo -e "plugins.xml done"

sed -i "s/more_email/$AUTHOR_EMAIL/g" ./package.xml || exit 1
sed -i "s/more_name/$AUTHOR_NAME/g" ./package.xml || exit 1
sed -i "s/more_project_name/$PROJECT_NAME/g" ./package.xml || exit 1
echo -e "package.xml done"

mv ./include/example.h "./include/$CLASS_NAME.h" || exit 1
sed -i "s/CLASS_NAME/$CLASS_NAME/g" "./include/$CLASS_NAME.h" || exit 1
sed -i "s/NAMESPACE_NAME/$NAMESPACE_NAME/g" "./include/$CLASS_NAME.h" || exit 1
echo -e "creating $CLASS_NAME.h done"

mv ./src/example.cpp "./src/$CLASS_NAME.cpp" || exit 1
sed -i "s/CLASS_NAME/$CLASS_NAME/g" "./src/$CLASS_NAME.cpp" || exit 1
sed -i "s/NAMESPACE_NAME/$NAMESPACE_NAME/g" "./src/$CLASS_NAME.cpp" || exit
echo -e "creating $CLASS_NAME.h done"

mv ./launch/example.launch "./launch/$PROJECT_NAME.launch" || exit 1
sed -i "s/CLASS_NAME/$CLASS_NAME/g" "./launch/$PROJECT_NAME.launch" || exit 1
sed -i "s/NAMESPACE_NAME/$NAMESPACE_NAME/g" "./launch/$PROJECT_NAME.launch" || exit 1
sed -i "s/PROJECT_NAME/$PROJECT_NAME/g" "./launch/$PROJECT_NAME.launch" || exit 1
echo -e "creating $CLASS_NAME.launch done\n"

rm -rf .git || exit 1
echo "Creating empty $PROJECT_NAME project finished. Now you can rename this directory, add it to git and start working."
