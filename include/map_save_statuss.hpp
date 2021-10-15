#ifndef MAP_SAVE_STATUSS_H
#define MAP_SAVE_STATUSS_H

namespace ros2_com
{
  enum class MapSaveStatuss
  {
    SHMEM_ERROR,
    YAML_ERROR,
    BIN_ERROR,
    NO_MAP_DATA,
    NO_MAP_PATH,
    SUCCESS
  };

}

#endif //MAP_SAVE_STATUSS_H