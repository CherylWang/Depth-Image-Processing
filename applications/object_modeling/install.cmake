include(../copy_file.cmake)
include(../copy_folder.cmake)

# Copy and Install HDF5 DLLs
copy_file("hdf5.dll" "${HDF5_INCLUDE_DIR}/../bin")
copy_file("hdf5_cpp.dll" "${HDF5_INCLUDE_DIR}/../bin")
copy_file("szip.dll" "${HDF5_INCLUDE_DIR}/../bin")
copy_file("zlib.dll" "${HDF5_INCLUDE_DIR}/../bin")

# Copy and Install GLUT DLLs
copy_file("freeglut.dll" "${GLUT_INCLUDE_DIR}/../bin")

# Copy and Install OpenNI2 DLLs
copy_file("OpenNI2.dll" "${OPENNI2_INCLUDE_DIRS}/../Redist")
copy_folder("OpenNI2" "${OPENNI2_INCLUDE_DIRS}/../Redist")
