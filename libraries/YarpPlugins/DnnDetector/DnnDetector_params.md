| Group |      Parameter      |  Type  | Units |          Default Value          | Required |            Description            |         Notes         |
|:-----:|:-------------------:|:------:|:-----:|:-------------------------------:|:--------:|:---------------------------------:|:---------------------:|
|       |    trainedModel     | string |       | yolov3-tiny/yolov3-tiny.weights |    no    |        trained model file         |                       |
|       |   configDNNModel    | string |       |   yolov3-tiny/yolov3-tiny.cfg   |    no    |        configuration file         |                       |
|       |     framework       | string |       |             darknet             |    no    |             framework             |                       |
|       | classesTrainedModel | string |       |   coco-object-categories.txt    |    no    |     file with trained classes     |                       |
|       |       backend       | string |       |              cuda               |    no    |              backend              | default, opencv, cuda |
|       |       target        | string |       |               cpu               |    no    |           target device           |   cpu, opencl, cuda   |
|       |        scale        | float  |       |             0.00392             |    no    |               scale               |                       |
|       |        mean         | double |       |               0.0               |    no    |               mean                |                       |
|       |    confThreshold    | float  |       |               0.1               |    no    |       confidence threshold        |                       |
|       |    nmsThreshold     | float  |       |               0.4               |    no    | non maximum suppression threshold |                       |
