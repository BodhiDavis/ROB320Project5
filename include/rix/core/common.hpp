#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <vector>
#include <map>
#include <string>

#include "rix/ipc/flags.hpp"
#include "rix/util/log.hpp"

#define RIX_HUB_PORT 48104
#define RIX_HUB_WEB_PORT 48109

namespace rix {
namespace core {

const std::string MACHINE_ID_FILE = "~/.rix/machine_id";

enum OPCODE {
    NODE_REGISTER = 80,  ///< Sends NODE_NOTIFY to Node
    SUB_REGISTER,        ///< Queues PUB_NOTIFY for Publishers and sends SUB_NOTIFY to Subscriber
    PUB_REGISTER,        ///< Queues SUB_NOTIFY for Subscribers and sends PUB_NOTIFY to Publisher
    SRV_REGISTER,        ///< No responses
    ACT_REGISTER,        ///< No responses

    SUB_NOTIFY = 90,
    PUB_NOTIFY,

    NODE_DEREGISTER =
        100,  ///< Queues PUB_NOTIFY with PubNotify for Publishers and sends SUB_NOTIFY with SubNotify to Subscriber
    SUB_DEREGISTER,  ///< Queues PUB_NOTIFY with PubNotify for Publishers and sends SUB_NOTIFY with SubNotify to
                     ///< Subscriber
    PUB_DEREGISTER,  ///< Queues SUB_NOTIFY with SubNotify for Subscribers and sends PUB_NOTIFY with PubNotify to
                     ///< Publisher
    SRV_DEREGISTER,  ///< No responses
    ACT_DEREGISTER,  ///< No responses

    SRV_REQUEST = 140,  ///< Sends SRV_RESPONSE with SrvInfo
    ACT_REQUEST,        ///< Sends ACT_RESPONSE with ActInfo
    SRV_RESPONSE = 150,
    ACT_RESPONSE,

    TERMINATE = 160
};

enum ERROR_CODE {
    NO_ERROR = 0,
    UNKNOWN_NODE_ID,
    DUPLICATE_NODE_ID,
    DUPLICATE_SUB_ID,
    DUPLICATE_PUB_ID,
    UNKNOWN_SRV,
    DUPLICATE_SRV_NAME,
    UNKNOWN_ACT,
    DUPLICATE_ACT_NAME,
    INVALID_MESSAGE_HASH,
    PROTOCOL_MISMATCH,
};

enum Protocol { TCP = 0x01, UDP = 0x02, SHM = 0x04, WEB = 0x08, WEB2 = 0x10 };
enum Type { PUBLISHER, SUBSCRIBER, SERVICE, ACTION };

}  // namespace core
}  // namespace rix