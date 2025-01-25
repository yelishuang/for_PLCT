from copr.v3 import config_from_file
from copr.v3 import BuildProxy

config = config_from_file()
build_proxy = BuildProxy(config)
project_name = "test"
#Get build information
build_info_list = build_proxy.get_list(
    ownername="listen_rain",
    projectname=project_name,
    packagename=None
)

# Loop through build info list and print source package and build state
for build in build_info_list:
    source_package = build['source_package']  # Accessing source package
    print(f"Source package: {source_package}")
    print("#############state:", build["state"])
