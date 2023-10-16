#pragma once

#include <array>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>
#include "mavlink_include.h"
#include "mavlink_command_receiver.h"
#include <boost/optional.hpp>
namespace mavsdk {

class MavsdkImpl;
class ServerComponentImpl;
class MavlinkCommandReceiver;

class MavlinkRequestMessageHandler {
public:
    MavlinkRequestMessageHandler() = delete;
    explicit MavlinkRequestMessageHandler(
        MavsdkImpl& mavsdk_impl,
        ServerComponentImpl& server_component_impl,
        MavlinkCommandReceiver& mavlink_command_receiver);
    ~MavlinkRequestMessageHandler();

    using Params = std::array<float, 5>;
    using Callback = std::function<boost::optional<MAV_RESULT>(
        uint8_t target_system_id, uint8_t target_component_id, const Params&)>;

    bool register_handler(uint32_t message_id, const Callback& callback, const void* cookie);
    void unregister_handler(uint32_t message_id, const void* cookie);
    void unregister_all_handlers(const void* cookie);

private:
    boost::optional<mavlink_message_t>
    handle_command_long(const MavlinkCommandReceiver::CommandLong& command);
    boost::optional<mavlink_message_t>
    handle_command_int(const MavlinkCommandReceiver::CommandInt& command);

    struct Entry {
        uint32_t message_id;
        Callback callback;
        const void* cookie;
    };

    std::mutex _table_mutex{};
    std::vector<Entry> _table{};

    MavsdkImpl& _mavsdk_impl;
    ServerComponentImpl& _server_component_impl;
    MavlinkCommandReceiver& _mavlink_command_receiver;
};

} // namespace mavsdk
