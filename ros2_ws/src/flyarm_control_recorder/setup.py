from setuptools import setup

package_name = "flyarm_control_recorder"
setup(name=package_name, version="0.1.0", packages=[package_name],
    data_files=[("share/ament_index/resource_index/packages", ["resource/"+package_name]),
                ("share/"+package_name, ["package.xml", "setup.py", "setup.cfg"])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Flyarm", maintainer_email="flyarm@example.invalid",
    description="Bounded durable typed-message recorder for offline/shadow phase one",
    license="Apache-2.0", tests_require=["pytest"],
    entry_points={"console_scripts": ["recorder_node = flyarm_control_recorder.recorder_node:main"]})
