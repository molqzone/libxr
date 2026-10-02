/**
 * @file test_topic.cpp
 * @brief Topic 发布与订阅测试 / Topic publication and subscription tests.
 *
 * 检查不同订阅方式收到的数据、时间戳和 ISR 标记，可变回调和队列满时的丢弃，以及初始化期
 * 类型契约冲突在所有构建类型中都是致命错误。
 * Check subscriber data, timestamps and ISR flags, mutable callbacks, drops when a
 * queue is full, and that init-time type contract conflicts are fatal in every build
 * type.
 */

#include <sys/wait.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "libxr.hpp"
#include "libxr_def.hpp"
#include "middleware/message/message_test_payloads.hpp"
#include "test.hpp"
#include "test_assert.hpp"

namespace
{

uint64_t TimestampUs(const LibXR::MicrosecondTimestamp& timestamp)
{
  return static_cast<uint64_t>(timestamp);
}

void TestTopicSubscriberDispatch()
{
  // 同一次发布送给不同种类的订阅者，检查数据、时间戳和回调收到的 ISR 标记。
  // Send one publication to different subscriber types and check data, timestamps and
  // callback ISR flags.
  TEST_ASSERT(LibXR::Topic::Find("missing_default_topic") == nullptr);

  auto domain = LibXR::Topic::Domain("message_topic_domain");
  auto topic = LibXR::Topic::CreateTopic<double>("message_topic_tp", &domain);
  static double msg[4];
  auto async_suber = LibXR::Topic::ASyncSubscriber<double>(topic);
  LibXR::SPSCQueue<double> msg_queue(10);
  auto queue_suber = LibXR::Topic::QueuedSubscriber(topic, msg_queue);
  LibXR::SPSCQueue<LibXR::Topic::Message<double>> timed_msg_queue(10);
  auto timed_queue_suber = LibXR::Topic::QueuedSubscriber(topic, timed_msg_queue);
  UNUSED(queue_suber);
  UNUSED(timed_queue_suber);

  static bool cb_in_isr = false;
  static LibXR::MicrosecondTimestamp cb_timestamp;
  static LibXR::MicrosecondTimestamp view_cb_timestamp;
  static double view_cb_value = 0.0;
  static LibXR::MicrosecondTimestamp raw_view_timestamp;
  static LibXR::MicrosecondTimestamp raw_arg_timestamp;
  static double raw_view_value = 0.0;
  static double raw_arg_value = 0.0;
  static double raw_mutable_arg_value = 0.0;
  static size_t raw_view_size = 0;
  static size_t raw_arg_size = 0;
  static size_t raw_mutable_arg_size = 0;

  auto msg_cb = LibXR::Topic::Callback::Create(
      [](bool in_isr, void*, LibXR::MicrosecondTimestamp timestamp, double& data)
      {
        cb_in_isr = in_isr;
        cb_timestamp = timestamp;
        msg[3] = data;
      },
      reinterpret_cast<void*>(0));
  topic.RegisterCallback(msg_cb);

  auto view_cb = LibXR::Topic::Callback::Create(
      [](bool, void*, const LibXR::Topic::MessageView<double>& message)
      {
        view_cb_timestamp = message.timestamp;
        TEST_ASSERT(message.data != nullptr);
        view_cb_value = *message.data;
      },
      reinterpret_cast<void*>(0));
  topic.RegisterCallback(view_cb);

  auto raw_view_cb = LibXR::Topic::Callback::Create(
      [](bool, void*, const LibXR::Topic::RawMessageView& message)
      {
        raw_view_timestamp = message.timestamp;
        raw_view_size = message.payload.size_;
        TEST_ASSERT(message.payload.addr_ != nullptr);
        raw_view_value = *static_cast<const double*>(message.payload.addr_);
      },
      reinterpret_cast<void*>(0));
  topic.RegisterCallback(raw_view_cb);

  auto raw_arg_cb = LibXR::Topic::Callback::Create(
      [](bool, void*, LibXR::MicrosecondTimestamp timestamp,
         const LibXR::ConstRawData& data)
      {
        raw_arg_timestamp = timestamp;
        raw_arg_size = data.size_;
        TEST_ASSERT(data.addr_ != nullptr);
        raw_arg_value = *static_cast<const double*>(data.addr_);
      },
      reinterpret_cast<void*>(0));
  topic.RegisterCallback(raw_arg_cb);

  auto raw_mutable_arg_cb = LibXR::Topic::Callback::Create(
      [](bool, void*, LibXR::ConstRawData& data)
      {
        raw_mutable_arg_size = data.size_;
        TEST_ASSERT(data.addr_ != nullptr);
        raw_mutable_arg_value = *static_cast<const double*>(data.addr_);
      },
      reinterpret_cast<void*>(0));
  topic.RegisterCallback(raw_mutable_arg_cb);

  TEST_ASSERT(!async_suber.Available());

  msg[0] = 16.16;
  const LibXR::MicrosecondTimestamp timestamp0(1001);
  async_suber.StartWaiting();
  topic.Publish(msg[0], timestamp0);
  TEST_ASSERT(async_suber.Available());
  TEST_ASSERT(async_suber.GetData() == msg[0]);
  TEST_ASSERT(TimestampUs(async_suber.GetTimestamp()) == TimestampUs(timestamp0));
  TEST_ASSERT(!async_suber.Available());
  TEST_ASSERT(msg_queue.Size() == 1);
  double queue_value = 0.0;
  msg_queue.Pop(queue_value);
  TEST_ASSERT(queue_value == msg[0]);
  TEST_ASSERT(timed_msg_queue.Size() == 1);
  LibXR::Topic::Message<double> queue_msg;
  timed_msg_queue.Pop(queue_msg);
  TEST_ASSERT(queue_msg.data == msg[0]);
  TEST_ASSERT(TimestampUs(queue_msg.timestamp) == TimestampUs(timestamp0));
  TEST_ASSERT(msg[3] == msg[0]);
  TEST_ASSERT(TimestampUs(cb_timestamp) == TimestampUs(timestamp0));
  TEST_ASSERT(view_cb_value == msg[0]);
  TEST_ASSERT(TimestampUs(view_cb_timestamp) == TimestampUs(timestamp0));
  TEST_ASSERT(raw_view_value == msg[0]);
  TEST_ASSERT(raw_view_size == sizeof(double));
  TEST_ASSERT(TimestampUs(raw_view_timestamp) == TimestampUs(timestamp0));
  TEST_ASSERT(raw_arg_value == msg[0]);
  TEST_ASSERT(raw_arg_size == sizeof(double));
  TEST_ASSERT(TimestampUs(raw_arg_timestamp) == TimestampUs(timestamp0));
  TEST_ASSERT(raw_mutable_arg_value == msg[0]);
  TEST_ASSERT(raw_mutable_arg_size == sizeof(double));
  TEST_ASSERT(!cb_in_isr);

  auto byte_stable_topic =
      LibXR::Topic::CreateTopic<ByteStablePayload>("byte_stable_tp", &domain);
  static LibXR::MicrosecondTimestamp byte_stable_view_timestamp;
  static float byte_stable_view_value = 0.0f;
  auto byte_stable_cb = LibXR::Topic::Callback::Create(
      [](bool, void*, const LibXR::Topic::MessageView<ByteStablePayload>& message)
      {
        byte_stable_view_timestamp = message.timestamp;
        TEST_ASSERT(message.data != nullptr);
        byte_stable_view_value = message.data->data[2];
      },
      reinterpret_cast<void*>(0));
  byte_stable_topic.RegisterCallback(byte_stable_cb);
  ByteStablePayload byte_stable_tx{1.0f, 2.0f, 3.0f, 4.0f};
  const LibXR::MicrosecondTimestamp byte_stable_timestamp(1501);
  byte_stable_topic.Publish(byte_stable_tx, byte_stable_timestamp);
  TEST_ASSERT(byte_stable_view_value == byte_stable_tx.data[2]);
  TEST_ASSERT(TimestampUs(byte_stable_view_timestamp) ==
              TimestampUs(byte_stable_timestamp));

  msg[0] = 32.32;
  msg[3] = -1.0f;
  const LibXR::MicrosecondTimestamp timestamp1(2002);
  async_suber.StartWaiting();
  topic.PublishFromCallback(msg[0], timestamp1, true);
  TEST_ASSERT(async_suber.Available());
  TEST_ASSERT(async_suber.GetData() == msg[0]);
  TEST_ASSERT(TimestampUs(async_suber.GetTimestamp()) == TimestampUs(timestamp1));
  TEST_ASSERT(msg_queue.Size() == 1);
  msg_queue.Pop(queue_value);
  TEST_ASSERT(queue_value == msg[0]);
  TEST_ASSERT(timed_msg_queue.Size() == 1);
  timed_msg_queue.Pop(queue_msg);
  TEST_ASSERT(queue_msg.data == msg[0]);
  TEST_ASSERT(TimestampUs(queue_msg.timestamp) == TimestampUs(timestamp1));
  TEST_ASSERT(msg[3] == msg[0]);
  TEST_ASSERT(TimestampUs(cb_timestamp) == TimestampUs(timestamp1));
  TEST_ASSERT(cb_in_isr);
  TEST_ASSERT(view_cb_value == msg[0]);
  TEST_ASSERT(TimestampUs(view_cb_timestamp) == TimestampUs(timestamp1));
  TEST_ASSERT(raw_view_value == msg[0]);
  TEST_ASSERT(raw_view_size == sizeof(double));
  TEST_ASSERT(TimestampUs(raw_view_timestamp) == TimestampUs(timestamp1));
  TEST_ASSERT(raw_arg_value == msg[0]);
  TEST_ASSERT(raw_arg_size == sizeof(double));
  TEST_ASSERT(TimestampUs(raw_arg_timestamp) == TimestampUs(timestamp1));
  TEST_ASSERT(raw_mutable_arg_value == msg[0]);
  TEST_ASSERT(raw_mutable_arg_size == sizeof(double));
}

void TestTopicMutationAndQueueDrop()
{
  // 可变回调修改调用者的数据；订阅队列满时丢弃新消息，保留先前排队的消息。
  // A mutable callback changes caller data; a full subscriber queue drops the new message
  // and keeps older ones.
  auto domain = LibXR::Topic::Domain("message_topic_mutation_domain");

  auto mutable_topic = LibXR::Topic::CreateTopic<int>("mutable_payload_tp", &domain);
  auto mutable_cb = LibXR::Topic::Callback::Create(
      [](bool, void*, int& data) { data = 5678; }, reinterpret_cast<void*>(0));
  mutable_topic.RegisterCallback(mutable_cb);
  int mutable_payload = 1234;
  mutable_topic.Publish(mutable_payload, LibXR::MicrosecondTimestamp(7037));
  TEST_ASSERT(mutable_payload == 5678);

  auto queue_drop_topic = LibXR::Topic::CreateTopic<int>("queue_drop_tp", &domain);
  LibXR::SPSCQueue<int> drop_queue(1);
  auto drop_suber = LibXR::Topic::QueuedSubscriber(queue_drop_topic, drop_queue);
  UNUSED(drop_suber);
  for (size_t i = 0; i < drop_queue.MaxSize(); ++i)
  {
    auto value = static_cast<int>(i);
    queue_drop_topic.Publish(value, LibXR::MicrosecondTimestamp(8000 + i));
  }
  TEST_ASSERT(drop_queue.Size() == drop_queue.MaxSize());
  int dropped_value = -123;
  queue_drop_topic.Publish(dropped_value, LibXR::MicrosecondTimestamp(9009));
  TEST_ASSERT(drop_queue.Size() == drop_queue.MaxSize());
  for (size_t i = 0; i < drop_queue.MaxSize(); ++i)
  {
    int value = 0;
    TEST_ASSERT(drop_queue.Pop(value) == LibXR::ErrorCode::OK);
    TEST_ASSERT(value == static_cast<int>(i));
  }
  int dropped_message = 0;
  TEST_ASSERT(drop_queue.Pop(dropped_message) == LibXR::ErrorCode::EMPTY);
}

constexpr int TOPIC_FATAL_EXIT = 42;

// 子进程的 STDIO 输出和交给父进程的系统管道写端。
// The child's STDIO output and the write end of the OS pipe to the parent.
LibXR::Pipe* topic_fatal_output = nullptr;
int topic_fatal_report_fd = -1;

template <typename Func>
void ExpectTopicFatal(const char* expected_message, Func&& func)
{
  // 在子进程里执行一次初始化操作。子进程没有 STDIO 输出线程，STDIO 改写到 Pipe；致命错误
  // 回调把其中的报错交给父进程，再以固定退出码结束。expected_message 为空表示不应出错。
  // Run one init operation in a child. The child has no STDIO output thread, so STDIO
  // writes to a Pipe; the fatal callback hands that message to the parent and exits with
  // a fixed code. A null expected_message means no error is expected.
  int report[2] = {-1, -1};
  TEST_ASSERT(pipe(report) == 0);
  std::fflush(nullptr);
  pid_t child = fork();
  TEST_ASSERT(child >= 0);

  if (child == 0)
  {
    close(report[0]);
    topic_fatal_report_fd = report[1];
    static LibXR::Pipe output(256);
    static LibXR::Mutex output_mutex;
    topic_fatal_output = &output;
    LibXR::STDIO::write_ = &output.GetWritePort();
    LibXR::STDIO::write_mutex_ = &output_mutex;
    LibXR::STDIO::write_stream_ = nullptr;
    auto cb = LibXR::Assert::FatalCallback::Create(
        [](bool, int code, const char*, uint32_t)
        {
          auto& read = topic_fatal_output->GetReadPort();
          uint8_t text[256] = {};
          const size_t size = read.Size();
          LibXR::ReadOperation op;
          if (size > 0 && read(LibXR::RawData{text, size}, op) == LibXR::ErrorCode::OK)
          {
            const ssize_t sent = write(topic_fatal_report_fd, text, size);
            UNUSED(sent);
          }
          _exit(code);
        },
        TOPIC_FATAL_EXIT);
    LibXR::Assert::RegisterFatalErrorCallback(cb);
    func();
    _exit(0);
  }

  close(report[1]);
  char message[256] = {};
  size_t length = 0;
  ssize_t got = 0;
  while ((got = read(report[0], message + length, sizeof(message) - 1 - length)) > 0)
  {
    length += static_cast<size_t>(got);
  }
  close(report[0]);

  int status = 0;
  TEST_ASSERT(waitpid(child, &status, 0) == child);
  TEST_ASSERT(WIFEXITED(status));
  TEST_ASSERT(WEXITSTATUS(status) == (expected_message ? TOPIC_FATAL_EXIT : 0));
  // 第一行是 topic 的报错；之后是致命错误处理打印的源码位置。
  // The first line is the topic message; the fatal handler's source location follows.
  const char* line_end = std::strstr(message, "\r\n");
  const size_t first_line =
      line_end ? static_cast<size_t>(line_end - message) + 2 : length;
  const char* expected = expected_message ? expected_message : "";
  TEST_ASSERT(first_line == std::strlen(expected));
  TEST_ASSERT(std::strncmp(message, expected, first_line) == 0);
}

}  // namespace

void test_message_topic()
{
  TestTopicSubscriberDispatch();
  TestTopicMutationAndQueueDrop();
}

void test_message_topic_contract()
{
  // 同名 topic 的类型或发布者约定冲突，以及订阅者和回调的类型不符，在 Debug 和 Release
  // 中都进入致命错误。
  // Type or publisher conflicts on an existing name, and subscriber or callback type
  // mismatches, are fatal in both Debug and Release.
  static_assert(sizeof(int64_t) == sizeof(double) && alignof(int64_t) == alignof(double));

  auto domain = LibXR::Topic::Domain("message_topic_contract_domain");
  auto topic = LibXR::Topic::CreateTopic<double>("contract_tp", &domain);
  const auto key = static_cast<unsigned>(LibXR::CRC32::Calculate("contract_tp", 11));

  // 报错写出 topic 名称（调用方有名称时）、CRC32 键和两边的字节数与对齐。
  // The message names the topic (when the call site has the name), its CRC32 key, and
  // both sizes and alignments.
  char named_float[160];
  std::snprintf(
      named_float, sizeof(named_float),
      "Topic type mismatch: topic \"contract_tp\" (key %u) carries 8 bytes align 8, "
      "requested 4 bytes align 4\r\n",
      key);
  char named_int64[160];
  std::snprintf(
      named_int64, sizeof(named_int64),
      "Topic type mismatch: topic \"contract_tp\" (key %u) carries 8 bytes align 8, "
      "requested 8 bytes align 8\r\n",
      key);
  char unnamed_float[160];
  std::snprintf(
      unnamed_float, sizeof(unnamed_float),
      "Topic type mismatch: topic key %u carries 8 bytes align 8, requested 4 bytes "
      "align 4\r\n",
      key);
  char callback_float[160];
  std::snprintf(
      callback_float, sizeof(callback_float),
      "Topic type mismatch: callback payload type differs from topic key %u (8 bytes "
      "align 8)\r\n",
      key);
  char single_publisher[160];
  std::snprintf(
      single_publisher, sizeof(single_publisher),
      "Topic publisher mismatch: topic \"contract_tp\" (key %u) was created for a "
      "single publisher, multi_publisher requested\r\n",
      key);

  ExpectTopicFatal(
      nullptr, [&] { (void)LibXR::Topic::FindOrCreate<double>("contract_tp", &domain); });
  ExpectTopicFatal(named_float, [&]
                   { (void)LibXR::Topic::CreateTopic<float>("contract_tp", &domain); });
  ExpectTopicFatal(
      named_int64,
      [&] { (void)LibXR::Topic::FindOrCreate<int64_t>("contract_tp", &domain); });
  ExpectTopicFatal(unnamed_float,
                   [&]
                   {
                     static float value = 0.0f;
                     LibXR::Topic::SyncSubscriber<float> suber(topic, value);
                     UNUSED(suber);
                   });
  ExpectTopicFatal(callback_float,
                   [&]
                   {
                     auto cb = LibXR::Topic::Callback::Create([](bool, void*, float&) {},
                                                              reinterpret_cast<void*>(0));
                     topic.RegisterCallback(cb);
                   });
  ExpectTopicFatal(
      single_publisher,
      [&] { (void)LibXR::Topic::CreateTopic<double>("contract_tp", &domain, true); });
  ExpectTopicFatal(
      single_publisher,
      [&] { (void)LibXR::Topic::FindOrCreate<double>("contract_tp", &domain, true); });
}
