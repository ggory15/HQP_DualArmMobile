#include <iostream>
#include "include/fcl_utility.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
#include "fcl/collision.h"
#include "fcl/BV/BV.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include <boost/filesystem.hpp>

#define TEST_RESOURCES_DIR "/home/kendrick/demo/fcl_ggory/fcl_resources"

using namespace fcl;

template<typename BV>
bool collide_Test(const Transform3f& tf,
                  const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method, bool verbose = true);

template<typename BV>
bool test_collide_func(const Transform3f& tf,
                       const std::vector<Vec3f>& vertices1, const std::vector<Triangle>& triangles1,
                       const std::vector<Vec3f>& vertices2, const std::vector<Triangle>& triangles2, SplitMethodType split_method);

int num_max_contacts = std::numeric_limits<int>::max();
bool enable_contact = true;

std::vector<Contact> global_pairs;
std::vector<Contact> global_pairs_now;

int main()
{
  std::vector<Vec3f> p1, p2;
  std::vector<Triangle> t1, t2;
  boost::filesystem::path path(TEST_RESOURCES_DIR);
  
  loadOBJFile((path / "env.obj").string().c_str(), p1, t1);
  loadOBJFile((path / "rob.obj").string().c_str(), p2, t2);

  FCL_REAL extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
  std::size_t n = 10;
  bool verbose = true;
  std::vector<Transform3f> transforms;
  generateRandomTransforms(extents, transforms, n);


  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    global_pairs.clear();
    global_pairs_now.clear();

    collide_Test<OBB>(transforms[i], p1, t1, p2, t2, SPLIT_METHOD_MEAN, verbose);
  }
  printf("finished\n");	
  return 0;
}

template<typename BV>
bool collide_Test(const Transform3f& tf,
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

  Transform3f pose1(tf), pose2;

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
      std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}


