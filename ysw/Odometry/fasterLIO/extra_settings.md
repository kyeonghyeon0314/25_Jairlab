1. /src/laser_mapping.cc
- line342: `        }   return;`
--> don't publish pointcloud to rviz

2. /app/run_mapping_online.cc
- comment line43~44; don't save pcd map

###### Optional

3. /rviz_cfg/loam_livox.rviz
- line83: `Enabled: false`
- line96: `Topic: /cloud_registered_body`
- line143: `Enabled: false`
- line160: `Value: false`
- line364~366: `X: 0, Y: 0, Z: 0`
- line373: `Target Frame: body`

4. /include/IKFoM_toolkit/mtk/types/wrapped_cv_mat.hpp
- line40: `#include <opencv2/opencv.hpp>`

5. vscode cpp config setting
- .vscode/