# 个人的任务文件

放本机自己的任务文件：数据集在哪、档案放哪，这些因人因机器而异。这个目录里除了本说明都不入库。

做法：复制 `configs/tasks/` 里的一份，改 `data.source.path` 等路径；`system` 写成相对本文件的路径，例如 `../systems/my_lio.yaml`。

    ./run.py configs/local/kitti00.yaml
