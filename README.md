JumpOverBlock is an efficient Line-Of-Sight checker for sparse grid/voxel maps.

Related aricle has been accept by IEEE RAL(https://ieeexplore.ieee.org/document/10266687)

1, Path to key elements

block detection: ~/algorithm/los_check_for_sparse/block_detector_greedy.h

Map down sample: ~/algorithm/los_check_for_sparse/map_down_sampler.h

LOS with jump over block: ~/algorithm/line_of_sight_jump_between_block.h

2, Path to unit test of JOB

2D block detection and visualization: ~/test/test_los_for_sparse_map.cpp

3D block detection and visualization: ~/test/test_3d_block_viewer.cpp 

3, Path to massive test of JOB

massive 2D and 3D LOS check and comparison: 
~/test/test_massive_los_check_with_jump_block.cpp

massive 2D, 3D and 4D LOS check of JOB, under random maps: 
~/test/test_massive_los_check_with_random_map.cpp

4, If you found its useful, please cite in

@ARTICLE{10266687,

  author={Yao, Zhuo and Wang, Wei and Zhang, Jiadong and Wang, Yan and Li, Jinjiang},

  journal={IEEE Robotics and Automation Letters}, 

  title={Jump Over Block (JOB): An Efficient Line-of-Sight Checker for Grid/Voxel Maps With Sparse Obstacles}, 

  year={2023},

  volume={8},

  number={11},

  pages={7575-7582},

  keywords={Path planning;Line-of-sight propagation;Partitioning algorithms;Planning;Collision avoidance;Toy manufacturing industry;Surface treatment;Collision avoidance;computational geometry;motion and path planning},

  doi={10.1109/LRA.2023.3320435}}
