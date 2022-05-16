option(DOWNLOAD_MODELS "Download DNN models for tests/examples" ON)

function(_download_model _dir _files)
    list(LENGTH _files _size)

    foreach(_i RANGE 1 ${_size} 2)
        math(EXPR _j "${_i} - 1")
        list(GET _files ${_j} _url)
        list(GET _files ${_i} _sha1)
        get_filename_component(_file ${_url} NAME)

        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/${_file}")
            file(SHA1 "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/${_file}" _sha1_actual)

            if(_sha1_actual STREQUAL _sha1)
                message(STATUS "File ${_dir}/${_file} already available")
                continue()
            else()
                message(STATUS "Removing ${_dir}/${_file} due to hash mismatch")
                file(REMOVE "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/${_file}")
            endif()
        endif()

        message(STATUS "Downloading ${_dir}/${_file}")

        file(DOWNLOAD ${_url} "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/${_file}"
             INACTIVITY_TIMEOUT 5
             STATUS _status
             TIMEOUT 30
             EXPECTED_HASH SHA1=${_sha1})

        if(NOT _status EQUAL 0)
            file(REMOVE "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}/${_file}")
            list(GET _status 0 _status_0)
            list(GET _status 1 _status_1)
            message(FATAL_ERROR "File download ${_dir}/${_file} returned ${_status_0}: ${_status_1}")
        endif()
    endforeach()
endfunction()

if(DOWNLOAD_MODELS)
    # format: set(_var [_url_1 _sha1_1] [_url_2 _sha1_2] ...)
    set(_yolov3-tiny https://pjreddie.com/media/files/yolov3-tiny.weights 40f3c11883bef62fd850213bc14266632ed4414f
                     https://raw.githubusercontent.com/pjreddie/darknet/17d4774/cfg/yolov3-tiny.cfg 46282cefe5cf7a248b69d528d3372f5cf1ddf39d)

    _download_model(yolov3-tiny "${_yolov3-tiny}")

    set(_shape_predictor_68_face_landmarks http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2 6cbb70006b91b31ba39a88594c2698abb5fdb785)

    _download_model(shape_predictor_68_face_landmarks "${_shape_predictor_68_face_landmarks}")

endif()
