1. /app/run_mapping_online.cc
- comment line43~45; disable saving map

2. /src/laser_mapping.cc
- comment line325, 326; inactivate printing log
- change line344 to: `publish_count_ -= options::PUBFRAME_PERIOD;//PublishFrameWorld();`
- change line350 to: `publish_count_ -= options::PUBFRAME_PERIOD;//PublishFrameEffectWorld(pub_laser_cloud_effect_world_);`

3. /rviz_cfg/loam_livox.rviz
- line96: `Topic: /cloud_registered_body`
- line143: `Enabled: false`
- line160: `Value: false`
- line364~366: `X: 0, Y: 0, Z: 0`
- line373: `Target Frame: body`