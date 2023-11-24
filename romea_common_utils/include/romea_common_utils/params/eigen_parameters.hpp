// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef ROMEA_COMMON_UTILS__PARAMS__EIGEN_PARAMETERS_HPP_
#define ROMEA_COMMON_UTILS__PARAMS__EIGEN_PARAMETERS_HPP_

// eigen
#include <Eigen/Core>

// std
#include <map>
#include <vector>
#include <memory>
#include <string>

// romea core
#include "romea_core_common/containers/Eigen/VectorOfEigenVector.hpp"
#include "romea_core_common/math/EulerAngles.hpp"
#include "romea_core_common/math/Transformation.hpp"

// local
#include "romea_common_utils/params/node_parameters.hpp"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  declare_vector_parameter<typename EigenVector::Scalar>(node, param_name);
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_vector_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_name,
  const EigenVector & default_values)
{
  using StdVector = std::vector<typename EigenVector::Scalar>;
  StdVector default_vector(default_values.data(), default_values.data() + default_values.size());

  declare_vector_parameter_with_default<typename EigenVector::Scalar>(
    node, param_name, default_vector);
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const std::string & param_name)
{
  declare_eigen_vector_parameter<EigenVector>(
    node, full_param_name(param_namespace, param_name));
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_vector_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const std::string & param_name,
  const EigenVector & default_values)
{
  declare_eigen_vector_parameter_with_default<EigenVector>(
    node, full_param_name(param_namespace, param_name), default_values);
}


//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline EigenVector get_eigen_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_name)
{
  return EigenVector(get_vector_parameter<typename EigenVector::Scalar>(node, param_name).data());
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline EigenVector get_eigen_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const std::string & param_name)
{
  return get_eigen_vector_parameter<EigenVector>(
    node, full_param_name(param_namespace, param_name));
}


//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_xyz_vector_parameter(std::shared_ptr<Node> node)
{
  declare_eigen_vector_parameter<EigenVector>(node, "xyz");
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_xyz_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace)
{
  declare_eigen_vector_parameter<EigenVector>(
    node, full_param_name(param_namespace, "xyz"));
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_xyz_vector_parameter_with_default(
  std::shared_ptr<Node> node,
  const EigenVector & default_values = EigenVector::Zero())
{
  declare_eigen_vector_parameter_with_default<EigenVector>(node, "xyz", default_values);
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_xyz_vector_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const EigenVector & default_values = EigenVector::Zero())
{
  declare_eigen_vector_parameter_with_default<EigenVector>(
    node, full_param_name(param_namespace, "xyz"), default_values);
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline EigenVector get_eigen_xyz_vector_parameter(std::shared_ptr<Node> node)
{
  return get_eigen_vector_parameter<EigenVector>(node, "xyz");
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline EigenVector get_eigen_xyz_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace)
{
  return get_eigen_vector_parameter<EigenVector>(
    node, full_param_name(param_namespace, "xyz"));
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_rpy_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace)
{
  declare_eigen_vector_parameter<EigenVector>(
    node, full_param_name(param_namespace, "rpy"));
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_rpy_vector_parameter_with_default(
  std::shared_ptr<Node> node, const EigenVector & default_values = EigenVector::Zero())
{
  declare_eigen_vector_parameter_with_default<EigenVector>(node, "rpy", default_values);
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_rpy_vector_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const EigenVector & default_values = EigenVector::Zero())
{
  declare_eigen_vector_parameter_with_default<EigenVector>(
    node, full_param_name(param_namespace, "rpy"), default_values);
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline EigenVector get_eigen_rpy_vector_parameter(std::shared_ptr<Node> node)
{
  return get_eigen_vector_parameter<EigenVector>(node, "rpy");
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline EigenVector get_eigen_rpy_vector_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace)
{
  return get_eigen_vector_parameter<EigenVector>(
    node, full_param_name(param_namespace, "rpy"));
}


//-----------------------------------------------------------------------------
template<typename EigenAffine, typename Node>
void declare_eigen_rigid_transformation_parameter(std::shared_ptr<Node> node)
{
  using Scalar = typename EigenAffine::Scalar;
  using EigenVector = Eigen::Matrix<Scalar, 3, 1>;

  declare_eigen_rpy_vector_parameter<EigenVector>(node);
  declare_eigen_xyz_vector_parameter<EigenVector>(node);
}

//-----------------------------------------------------------------------------
template<typename EigenAffine, typename Node>
void declare_eigen_rigid_transformation_parameter_with_default(
  std::shared_ptr<Node> node, const EigenAffine & default_value = EigenAffine::Identity())
{
  using Scalar = typename EigenAffine::Scalar;
  using EigenVector = Eigen::Matrix<Scalar, 3, 1>;

  auto translation = default_value.translation();
  auto angles = rotation3DToEulerAngles(default_value.rotation());
  declare_eigen_rpy_vector_parameter_with_default<EigenVector>(node, angles);
  declare_eigen_xyz_vector_parameter_with_default<EigenVector>(node, translation);
}

//-----------------------------------------------------------------------------
template<typename EigenAffine, typename Node>
void declare_eigen_rigid_transformation_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace)
{
  using Scalar = typename EigenAffine::Scalar;
  using EigenVector = Eigen::Matrix<Scalar, 3, 1>;

  declare_eigen_rpy_vector_parameter<EigenVector>(node, param_namespace);
  declare_eigen_xyz_vector_parameter<EigenVector>(node, param_namespace);
}

//-----------------------------------------------------------------------------
template<typename EigenAffine, typename Node>
void declare_eigen_rigid_transformation_parameter_with_default(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const EigenAffine & default_value = EigenAffine::Identity())
{
  using Scalar = typename EigenAffine::Scalar;
  using EigenVector = Eigen::Matrix<Scalar, 3, 1>;

  auto translation = default_value.translation();
  auto angles = rotation3DToEulerAngles(default_value.rotation());
  declare_eigen_rpy_vector_parameter_with_default<EigenVector>(node, param_namespace, angles);
  declare_eigen_xyz_vector_parameter_with_default<EigenVector>(node, param_namespace, translation);
}

//-----------------------------------------------------------------------------
template<typename EigenAffine, typename Node>
inline EigenAffine get_eigen_rigid_transformation_parameter(
  std::shared_ptr<Node> node,
  const std::string & param_namespace)
{
  using Scalar = typename EigenAffine::Scalar;
  using EigenVector = Eigen::Matrix<Scalar, 3, 1>;

  auto rotation = get_eigen_rpy_vector_parameter<EigenVector>(node, param_namespace);
  auto translation = get_eigen_xyz_vector_parameter<EigenVector>(node, param_namespace);
  return core::rigid_transformation3<Scalar>(translation, rotation);
}


//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_vector_parameters(
  std::shared_ptr<Node> node,
  const std::string & params_namespace,
  const std::vector<std::string> & params_names)
{
  using StdVector = std::vector<typename EigenVector::Scalar>;
  declare_parameters<StdVector>(node, params_namespace, params_names);
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_vector_parameters_with_default(
  std::shared_ptr<Node> node,
  const std::string & params_namespace,
  const std::vector<std::string> & params_names,
  const EigenVector & default_values = EigenVector::Zero())
{
  using StdVector = std::vector<typename EigenVector::Scalar>;
  StdVector default_vector(default_values.data(), default_values.data() + default_values.size());
  declare_parameters_with_default<StdVector>(node, params_namespace, params_names, default_vector);
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline void declare_eigen_vector_parameters_with_default(
  std::shared_ptr<Node> node,
  const std::string & params_namespace,
  const std::vector<std::string> & params_names,
  const std::vector<EigenVector, Eigen::aligned_allocator<EigenVector>> & default_values)
{
  assert(params_names.size() == default_values.size());
  using StdVector = std::vector<typename EigenVector::Scalar>;

  for (size_t i = 0; i < params_names.size(); ++i) {
    StdVector default_vector(default_values[i].data(),
      default_values[i].data() + default_values[i].size());

    declare_parameter_with_default<StdVector>(
      node, params_namespace, params_names[i],
      default_values[i]);
  }
}

//-----------------------------------------------------------------------------
template<typename EigenVector, typename Node>
inline std::vector<EigenVector, Eigen::aligned_allocator<EigenVector>> get_eigen_vector_parameters(
  std::shared_ptr<Node> node,
  const std::string & param_namespace,
  const std::vector<std::string> & param_names)
{
  using VectorOfEigenVector = std::vector<EigenVector, Eigen::aligned_allocator<EigenVector>>;
  using StdVector = std::vector<typename EigenVector::Scalar>;
  using VectorOfStdVector = std::vector<StdVector>;

  VectorOfEigenVector eigen_parameters(param_names.size());
  VectorOfStdVector std_parameters = get_parameters<StdVector>(node, param_namespace, param_names);

  for (size_t i = 0; i < std_parameters.size(); ++i) {
    eigen_parameters[i] = EigenVector(std_parameters[i].data());
  }

  return eigen_parameters;
}

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_COMMON_UTILS__PARAMS__EIGEN_PARAMETERS_HPP_
