// Read-only transport client. SetState below affects only this process's ECM.
// It never starts Gazebo, publishes a command, or loads a system.
#include <gz/transport/Node.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gs = gz::sim;
namespace gc = gz::sim::components;
struct Binding {
  gs::Entity world, model, link;
};

static bool safe(const std::string &s) {
  return std::regex_match(s, std::regex("[A-Za-z_][A-Za-z0-9_-]*"));
}

static Binding verify(const gz::msgs::SerializedStepMap &reply,
                      const std::string &worldName,
                      const std::string &modelName,
                      const std::string &linkName) {
  const std::set<gs::ComponentTypeId> allowed{
    gc::World::typeId, gc::Name::typeId, gc::Model::typeId,
    gc::ParentEntity::typeId, gc::Link::typeId};
  gz::msgs::SerializedStateMap subset;
  for (const auto &entry : reply.state().entities()) {
    const auto &entity = entry.second;
    if (entry.first == 0 || (entity.id() != 0 && entity.id() != entry.first) || entity.remove())
      throw std::runtime_error("Malformed or removed entity in full ECS snapshot");
    auto &copy = (*subset.mutable_entities())[entry.first];
    copy.set_id(entry.first);
    for (const auto &component : entity.components()) {
      const auto &value = component.second;
      const auto type = static_cast<std::uint64_t>(component.first);
      if (value.type() != 0 && type != value.type())
        throw std::runtime_error("ECS component map key/type mismatch");
      if (!allowed.count(type)) continue;
      if (value.remove())
        throw std::runtime_error("Removed identity component in full ECS snapshot");
      (*copy.mutable_components())[component.first] = value;
      (*copy.mutable_components())[component.first].set_type(type);
    }
  }
  gs::EntityComponentManager local;
  local.SetState(subset);  // This is a local copy, not a Gazebo write API.
  std::vector<gs::Entity> worlds, models, links;
  local.Each<gc::World, gc::Name>([&](const gs::Entity &id,
      const gc::World *, const gc::Name *name) {
    if (name->Data() == worldName) worlds.push_back(id);
    return true;
  });
  local.Each<gc::Model, gc::Name>([&](const gs::Entity &id,
      const gc::Model *, const gc::Name *name) {
    if (name->Data() == modelName) models.push_back(id);
    return true;
  });
  if (worlds.size() != 1 || models.size() != 1)
    throw std::runtime_error("ECS must identify exactly one expected World and Model");
  const auto parent = local.Component<gc::ParentEntity>(models.front());
  if (!parent || parent->Data() != worlds.front())
    throw std::runtime_error("Actual model is not a direct child of the expected World");
  local.Each<gc::Link, gc::Name, gc::ParentEntity>([&](const gs::Entity &id,
      const gc::Link *, const gc::Name *name, const gc::ParentEntity *owner) {
    if (name->Data() == linkName && owner->Data() == models.front()) links.push_back(id);
    return true;
  });
  if (links.size() != 1)
    throw std::runtime_error("ECS must identify exactly one actual model-owned base link");
  if (worlds.front() == models.front() || worlds.front() == links.front() ||
      models.front() == links.front())
    throw std::runtime_error("World/model/link identities must be distinct");
  return {worlds.front(), models.front(), links.front()};
}

static gz::msgs::SerializedStepMap synthetic(bool duplicateWorld = false,
                                           bool wrongModelParent = false,
                                           bool wrongLinkParent = false) {
  gs::EntityComponentManager ecm;
  ecm.SetEntityCreateOffset(42); // Test identity need not be world ID 1.
  const auto world = ecm.CreateEntity();
  ecm.CreateComponent(world, gc::World());
  ecm.CreateComponent(world, gc::Name("test_world"));
  const auto other = ecm.CreateEntity();
  if (duplicateWorld) {
    ecm.CreateComponent(other, gc::World());
    ecm.CreateComponent(other, gc::Name("test_world"));
  }
  const auto model = ecm.CreateEntity();
  ecm.CreateComponent(model, gc::Model());
  ecm.CreateComponent(model, gc::Name("test_model_0"));
  ecm.CreateComponent(model, gc::ParentEntity(wrongModelParent ? other : world));
  const auto link = ecm.CreateEntity();
  ecm.CreateComponent(link, gc::Link());
  ecm.CreateComponent(link, gc::Name("base_link"));
  ecm.CreateComponent(link, gc::ParentEntity(wrongLinkParent ? world : model));
  gz::msgs::SerializedStepMap result;
  ecm.State(*result.mutable_state(), {}, {}, true);
  return result;
}

static void selfTest() {
  const auto good = verify(synthetic(), "test_world", "test_model_0", "base_link");
  if (good.world == 1) throw std::runtime_error("Synthetic world-ID test invalid");
  int rejected = 0;
  for (const auto &bad : {synthetic(true), synthetic(false, true), synthetic(false, false, true)}) {
    try { verify(bad, "test_world", "test_model_0", "base_link"); }
    catch (const std::exception &) { ++rejected; }
  }
  try { verify(synthetic(), "wrong_world", "test_model_0", "base_link"); }
  catch (const std::exception &) { ++rejected; }
  try { verify(synthetic(), "test_world", "missing_model", "base_link"); }
  catch (const std::exception &) { ++rejected; }
  if (rejected != 5) throw std::runtime_error("Identity rejection self-test failed");
  std::cout << "{\"passed\":true,\"checks\":6,\"live_transport_used\":false}\n";
}

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") { selfTest(); return 0; }
    std::map<std::string, std::string> args;
    for (int i = 1; i < argc; i += 2) {
      if (i + 1 == argc || !args.emplace(argv[i], argv[i+1]).second)
        throw std::runtime_error("Expected unique option/value pairs");
    }
    const std::set<std::string> keys{"--world", "--model", "--link", "--timeout-ms", "--state-output"};
    for (const auto &item : args) if (!keys.count(item.first))
      throw std::runtime_error("Unknown option: " + item.first);
    const auto world = args.at("--world"), model = args.at("--model"), link = args.at("--link");
    if (!safe(world) || !safe(model) || !safe(link)) throw std::runtime_error("Unsafe entity name");
    const auto path = std::filesystem::path(args.at("--state-output"));
    if (!path.is_absolute() || !std::filesystem::is_directory(path.parent_path()) || std::filesystem::exists(path))
      throw std::runtime_error("A new absolute state-output file in an existing directory is required");
    const std::string rawTimeout = args.count("--timeout-ms") ? args.at("--timeout-ms") : "5000";
    std::size_t end = 0;
    const int timeout = std::stoi(rawTimeout, &end);
    if (end != rawTimeout.size() || timeout < 100 || timeout > 10000)
      throw std::runtime_error("timeout-ms must be an integer from 100 to 10000");
    const auto service = "/world/" + world + "/state";
    gz::transport::Node node;
    gz::msgs::Empty request;
    gz::msgs::SerializedStepMap reply;
    bool result = false;
    const bool executed = node.Request(service, request, timeout, reply, result);
    if (!executed || !result) throw std::runtime_error("Read-only world state request failed");
    const auto found = verify(reply, world, model, link);
    std::ofstream state(path, std::ios::binary | std::ios::out);
    if (!state || !reply.SerializeToOstream(&state)) throw std::runtime_error("Cannot write ECS snapshot evidence");
    state.close();
    if (!state) throw std::runtime_error("ECS evidence flush failed");
    std::cout << "{\"schema\":\"gz_ecs_world_entity_binding_v1\",\"verified\":true,"
      << "\"read_only\":true,\"api\":\"gz_sim8_SerializedStepMap_local_ECM\","
      << "\"world\":\"" << world << "\",\"world_id\":" << found.world
      << ",\"model\":\"" << model << "\",\"model_id\":" << found.model
      << ",\"model_parent_id\":" << found.world
      << ",\"link\":\"" << link << "\",\"link_id\":" << found.link
      << ",\"link_parent_id\":" << found.model
      << ",\"service\":\"" << service << "\",\"component_types\":{"
      << "\"World\":" << gc::World::typeId << ",\"Name\":" << gc::Name::typeId
      << ",\"Model\":" << gc::Model::typeId << ",\"Link\":" << gc::Link::typeId
      << ",\"ParentEntity\":" << gc::ParentEntity::typeId << "},"
      << "\"sim_sec\":" << reply.stats().sim_time().sec()
      << ",\"sim_nsec\":" << reply.stats().sim_time().nsec() << "}\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "world_entity_probe: " << e.what() << '\n'; return 1;
  }
}
