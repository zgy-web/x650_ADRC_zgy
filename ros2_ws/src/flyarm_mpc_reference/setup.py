from setuptools import setup

package_name = "flyarm_mpc_reference"
setup(name=package_name, version="0.1.0", packages=[package_name],
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"])],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="Flyarm maintainers", maintainer_email="maintainer@example.invalid",
      description="Offline/shadow Python reference MPC process", license="Apache-2.0",
      entry_points={"console_scripts": ["reference_node = flyarm_mpc_reference.node:main"]})
