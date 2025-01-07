add_library(trajectory Trajectory.cpp Path.cpp)
add_library(scene SceneModel.cpp XmlReader.cpp UrdfReader.cpp MeshFileIO.cpp)

AddTutorials(tutorial trajectory)