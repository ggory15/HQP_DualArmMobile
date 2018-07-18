#include "include/fcl/fcl_model.h"
#define TEST_RESOURCES_DIR "/home/kendrick/robostar_fcl/fcl_resources"
using namespace fcl;

int num_max_contacts = std::numeric_limits<int>::max();
bool enable_contact = true;

std::vector<Contact> global_pairs;
std::vector<Contact> global_pairs_now;


template<typename BV>
bool collide_Test(const Transform3f& tf1,const Transform3f& tf2,
                  const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;

  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3f pose1(tf1), pose2(tf2);

  CollisionResult local_result;

  MeshCollisionTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, pose1, m2, pose2,
                     CollisionRequest(num_max_contacts, enable_contact), local_result))
    std::cout << "initialize error" << std::endl;



  node.enable_statistics = verbose;

  collide(&node);

  if(local_result.numContacts() > 0)
  {
    if(global_pairs.size() == 0)
    {
      local_result.getContacts(global_pairs);
      std::sort(global_pairs.begin(), global_pairs.end());
    }
    else
    {
      local_result.getContacts(global_pairs_now);
      std::sort(global_pairs_now.begin(), global_pairs_now.end());
    }

    if(verbose)
  //    std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
  //  if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    //if(verbose) std::cout << "collision free " << std::endl;
    //if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}
template<typename BV>
void distance_Test(const Transform3f& tf1, const Transform3f& tf2,
                   const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method,
                   int qsize,
                   DistanceRes& distance_result,
                   bool verbose)
{
  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new BVSplitter<BV>(split_method));


  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3f pose1(tf1), pose2(tf2);

  DistanceResult local_result;
  MeshDistanceTraversalNode<BV> node;

  if(!initialize<BV>(node, m1, pose1, m2, pose2, DistanceRequest(true), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  distance(&node, NULL, qsize);
  distance_result.distance = local_result.min_distance;
  distance_result.p1 = local_result.nearest_points[0];
  distance_result.p2 = local_result.nearest_points[1];

  // if(verbose)
  // {
  //   std::cout << "distance " << local_result.min_distance << std::endl;

  //   std::cout << local_result.nearest_points[0][0] << " " << local_result.nearest_points[0][1] << " " << local_result.nearest_points[0][2] << std::endl;
  //   std::cout << local_result.nearest_points[1][0] << " " << local_result.nearest_points[1][1] << " " << local_result.nearest_points[1][2] << std::endl;
  //   std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
  // }
}



void FCL_MODEL::Initialize_Model()
{

  boost::filesystem::path path(TEST_RESOURCES_DIR);
  
  loadOBJFile((path / "Link1.obj").string().c_str(), p[0], t[0]);
  loadOBJFile((path / "Link2.obj").string().c_str(), p[1], t[1]);
  loadOBJFile((path / "Link3.obj").string().c_str(), p[2], t[2]);
  loadOBJFile((path / "Link4.obj").string().c_str(), p[3], t[3]);
  loadOBJFile((path / "Link5.obj").string().c_str(), p[4], t[4]);
  loadOBJFile((path / "Link6.obj").string().c_str(), p[5], t[5]);
  loadOBJFile((path / "Link7.obj").string().c_str(), p[6], t[6]);


}

void FCL_MODEL::Check_Collision_Pairs()
{


  global_pairs.clear();
  global_pairs_now.clear();

  bool verbose = true;
  bool verbose_[10] ;

  for (int i=0;i<10;i++)
   verbose_[i] = true;
  
  bool pair[15];
  bool dis_pair[15];

  for (int i=0;i<7;i++)
    tf[i].setTransform(Rot[i], Trs[i]);


  // for (int j=0;j<7;j++)
  //   for(int k=0;k<7;k++)
  //    pair[j+k] = collide_Test<OBB>(tf[j], tf[k],p[j], t[j], p[k], t[k], SPLIT_METHOD_MEAN, verbose);
  // 거리를 계산하고 있다가 > 충돌 체크??

  distance_Test<OBBRSS>(tf[0], tf[4], p[0], t[0], p[4], t[4], SPLIT_METHOD_MEAN,20, Dis[0],verbose_[0]);
  
  //pair[0] = collide_Test<OBB>(tf[0], tf[4], p[0], t[0], p[4], t[4], SPLIT_METHOD_MEAN, verbose);
  
  //global_pairs.clear();
  //global_pairs_now.clear();

  distance_Test<OBBRSS>(tf[0], tf[5], p[0], t[0], p[5], t[5], SPLIT_METHOD_MEAN,20, Dis[1],verbose_[1]);
  // pair[1] = collide_Test<OBB>(tf[0], tf[5], p[0], t[0], p[5], t[5], SPLIT_METHOD_MEAN, verbose);
  // global_pairs.clear();
  // global_pairs_now.clear();

 // distance_Test<OBBRSS>(tf[0], tf[6], p[0], t[0], p[6], t[6], SPLIT_METHOD_MEAN,20, Dis[2],verbose_[2]);
  // pair[2] = collide_Test<OBB>(tf[0], tf[6], p[0], t[0], p[6], t[6], SPLIT_METHOD_MEAN, verbose);
  // global_pairs.clear();
  // global_pairs_now.clear();

  distance_Test<OBBRSS>(tf[1], tf[4], p[1], t[1], p[4], t[4], SPLIT_METHOD_MEAN,20, Dis[2],verbose_[2]);
  // pair[3] = collide_Test<OBB>(tf[1], tf[4], p[1], t[1], p[4], t[4], SPLIT_METHOD_MEAN, verbose);
  // global_pairs.clear();
  // global_pairs_now.clear();

  distance_Test<OBBRSS>(tf[1], tf[5], p[1], t[1], p[5], t[5], SPLIT_METHOD_MEAN,20, Dis[3],verbose_[3]);
  // pair[4] = collide_Test<OBB>(tf[1], tf[5], p[1], t[1], p[5], t[5], SPLIT_METHOD_MEAN, verbose);
  // global_pairs.clear();
  // global_pairs_now.clear();

//  distance_Test<OBBRSS>(tf[1], tf[6], p[1], t[1], p[6], t[6], SPLIT_METHOD_MEAN,20, Dis[5],verbose_[5]);
  // pair[5] = collide_Test<OBB>(tf[1], tf[6], p[1], t[1], p[6], t[6], SPLIT_METHOD_MEAN, verbose);
  // global_pairs.clear();
  // global_pairs_now.clear();
 // distance_Test<OBBRSS>(tf[0], tf[5], p[0], t[0], p[5], t[5], SPLIT_METHOD_MEAN,20, Dis[3],verbose_[3]);


  // pair[6] = collide_Test<OBB>(tf[2], tf[6], p[2], t[2], p[6], t[6], SPLIT_METHOD_MEAN, verbose);
  // global_pairs.clear();
  // global_pairs_now.clear();


  // if (pair[0])
  //   cout << "Collision between link 1 and link 5" << endl;
  
  // if(pair[1])
  //   cout << "Collision between link 1 and link 6" << endl;

  // if(pair[2])
  //   cout << "Collision between link 1 and link 7" << endl;
    
  // if(pair[3])
  //   cout << "Collision between link 2 and link 5" << endl;

  // if(pair[4])
  //   cout << "Collision between link 2 and link 6" << endl;

  // if(pair[5])
  //   cout << "Collision between link 2 and link 7" << endl;

  // if(pair[6])
  //   cout << "Collision between link 3 and link 7" << endl;



}
