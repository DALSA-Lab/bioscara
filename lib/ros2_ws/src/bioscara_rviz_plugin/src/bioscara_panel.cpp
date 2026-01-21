/* Author: sbstorz */

#include <bioscara_rviz_plugin/bioscara_panel.hpp>
#include <rviz_common/display_context.hpp>
#include <chrono>

#include "ui_bioscara_rviz_plugin_frame.h"

namespace bioscara_rviz_plugin
{
  BioscaraPanel::BioscaraPanel(QWidget *parent) : Panel(parent)
  {
    ui_ = new Ui::BioscaraUI();
    ui_->setupUi(this);

    // TODO: since controller and hardware cb's are very similar, call lambda function with argument
    // As for the homing buttons
    connect(ui_->arm_en_btn, SIGNAL(clicked()), this, SLOT(arm_en_btn_cb()));
    connect(ui_->gripper_en_btn, SIGNAL(clicked()), this, SLOT(gripper_en_btn_cb()));
    connect(ui_->vjtc_ctrl_en_btn, SIGNAL(clicked()), this, SLOT(vjtc_ctrl_en_btn_cb()));
    connect(ui_->homing_ctrl_en_btn, SIGNAL(clicked()), this, SLOT(homing_ctrl_en_btn_cb()));
    connect(ui_->gripper_ctrl_en_btn, SIGNAL(clicked()), this, SLOT(gripper_ctrl_en_btn_cb()));

    connect(ui_->j1_hm_neg_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j1", -1); });
    connect(ui_->j1_hm_stp_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j1", 0); });
    connect(ui_->j1_hm_pos_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j1", 1); });
    connect(ui_->j2_hm_neg_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j2", -1); });
    connect(ui_->j2_hm_stp_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j2", 0); });
    connect(ui_->j2_hm_pos_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j2", 1); });
    connect(ui_->j3_hm_neg_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j3", -1); });
    connect(ui_->j3_hm_stp_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j3", 0); });
    connect(ui_->j3_hm_pos_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j3", 1); });
    connect(ui_->j4_hm_neg_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j4", -1); });
    connect(ui_->j4_hm_stp_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j4", 0); });
    connect(ui_->j4_hm_pos_btn, &QPushButton::clicked, this, [this]
            { homing_cmd("j4", 1); });
  }

  // TODO: manually delete ui_ object?
  BioscaraPanel::~BioscaraPanel() = default;

  void BioscaraPanel::onInitialize()
  {

    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    // Timer which cleans up all requests from the service clients which have not been answered.
    using namespace std::chrono_literals;
    prune_timer_ = node_->create_wall_timer(
        5s,
        [this]()
        {
          // Prune all requests older than 5s.
          size_t n_pruned = switch_controller_client_->prune_requests_older_than(
              std::chrono::system_clock::now() - 5s);
          n_pruned += configure_controller_client_->prune_requests_older_than(
              std::chrono::system_clock::now() - 5s);
          n_pruned += hardware_state_client_->prune_requests_older_than(
              std::chrono::system_clock::now() - 5s);
          if (n_pruned)
          {
            RCLCPP_INFO(
                node_->get_logger(),
                "The server hasn't replied for more than 5s, %zu requests were discarded", n_pruned);
          }
        });

    cm_state_subsription_ =
        node_->create_subscription<ControllerManagerActivity>(
            "/controller_manager/activity", rclcpp::QoS(10).transient_local().reliable(),
            std::bind(&BioscaraPanel::cm_state_callback, this, std::placeholders::_1));

    prepopulate_state_map(controller_states_, {"velocity_joint_trajectory_controller",
                                               "gripper_controller",
                                               "homing_controller",
                                               "joint_state_broadcaster"});

    prepopulate_state_map(hardware_states_, {"bioscara_arm",
                                             "bioscara_gripper_128"});

    joint_state_subsription_ =
        node_->create_subscription<DynamicJointState>(
            "/dynamic_joint_states", rclcpp::QoS(10).transient_local().best_effort(),
            std::bind(&BioscaraPanel::joint_state_callback, this, std::placeholders::_1));

    prepopulate_joint_state_map(joint_states_, {"j1", "j2", "j3", "j4", "gripper"});

    homing_publisher_ = node_->create_publisher<DynamicInterfaceGroupValues>("/homing_controller/commands", 10);

    hardware_state_client_ = node_->create_client<SetHardwareComponentState>(
        "/controller_manager/set_hardware_component_state");

    switch_controller_client_ = node_->create_client<SwitchController>(
        "/controller_manager/switch_controller");

    configure_controller_client_ = node_->create_client<ConfigureController>(
        "/controller_manager/configure_controller");
  }

  void BioscaraPanel::cm_state_callback(const ControllerManagerActivity &msg)
  {
    named_lcs_msg_to_map(msg.controllers, controller_states_);
    named_lcs_msg_to_map(msg.hardware_components, hardware_states_);

    print_cm_map("New controller states:", controller_states_);
    print_cm_map("New hardware states:", hardware_states_);

    ensure_jsb_is_active();

    update_state_labels_and_btns();
    update_homing_grp_state();
  }

  void BioscaraPanel::joint_state_callback(const DynamicJointState &msg)
  {
    dynamic_joint_state_msg_to_map(msg, joint_states_);

    update_homing_state_labels();
  }

  void BioscaraPanel::ensure_jsb_is_active(void)
  {
    if (controller_states_.at("joint_state_broadcaster").state.id !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      set_controller_state("joint_state_broadcaster",
                           lifecycle_msgs::msg::State().set__id(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
    }
  }

  void BioscaraPanel::set_hardware_component_state(const std::string component, const lifecycle_msgs::msg::State target_state)
  {
    auto req = std::make_shared<SetHardwareComponentState::Request>();
    req->name = component;
    req->target_state = target_state;

    using ServiceResponseFuture = rclcpp::Client<SetHardwareComponentState>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
    {
      auto result = future.get();
      RCLCPP_INFO(node_->get_logger(), "Called 'set_hardware_component_state': %s", result->ok ? "Success" : "Failure");
    };

    auto result_future = hardware_state_client_->async_send_request(req, response_received_callback);
  }

  void BioscaraPanel::configure_controller(const std::string controller)
  {
    auto req = std::make_shared<ConfigureController::Request>();
    req->name = controller;

    using ServiceResponseFuture = rclcpp::Client<ConfigureController>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
    {
      auto result = future.get();
      RCLCPP_INFO(node_->get_logger(), "Called 'configure_controller': %s", result->ok ? "Success" : "Failure");
    };

    auto result_future = configure_controller_client_->async_send_request(req, response_received_callback);
  }

  void BioscaraPanel::switch_controllers(const std::vector<std::string> &activate_controllers,
                                         const std::vector<std::string> &deactivate_controllers,
                                         const int32_t strictness,
                                         const bool activate_asap,
                                         const builtin_interfaces::msg::Duration timeout)
  {
    auto req = std::make_shared<SwitchController::Request>();
    req->activate_controllers = activate_controllers;
    req->deactivate_controllers = deactivate_controllers;
    req->strictness = strictness;
    req->activate_asap = activate_asap;
    req->timeout = timeout;

    using ServiceResponseFuture = rclcpp::Client<SwitchController>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
    {
      auto result = future.get();
      RCLCPP_INFO(node_->get_logger(), "Called 'switch_controllers': %s: '%s'", result->ok ? "Success" : "Failure", result->message.c_str());
    };

    auto result_future = switch_controller_client_->async_send_request(req, response_received_callback);
  }

  void BioscaraPanel::set_controller_state(const std::string controller,
                                           const lifecycle_msgs::msg::State target_state)
  {
    lifecycle_msgs::msg::State current_state = controller_states_.at(controller).state;
    switch (current_state.id)
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      switch (target_state.id)
      {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        switch_controllers({}, {controller});
        return;

      default:
        break;
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      switch (target_state.id)
      {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        switch_controllers({controller}, {});
        return;

      default:
        break;
      }
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      switch (target_state.id)
      {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        configure_controller(controller);
        switch_controllers({controller}, {});
        return;

      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
         configure_controller(controller);
        return;

      default:
        break;
      }
      break;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Can not set controller state to target state, currently not supported.");
      break;
    }
  }

  lifecycle_msgs::msg::State BioscaraPanel::target_state_from_current(lifecycle_msgs::msg::State current_state)
  {
    lifecycle_msgs::msg::State target_state;
    switch (current_state.id)
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
      break;

    default:
      target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
      break;
    }
    return target_state;
  }

  void BioscaraPanel::homing_cmd(const std::string joint, const int cmd)
  {
    RCLCPP_INFO(node_->get_logger(), "Homing %s, %d", joint.c_str(), cmd);

    DynamicInterfaceGroupValues msg;
    InterfaceValue value;
    value.interface_names = {"home"};
    value.values = {cmd * 1.0};
    msg.interface_groups = {joint};
    msg.interface_values = {value};

    homing_publisher_->publish(msg);
  }

  void BioscaraPanel::arm_en_btn_cb(void)
  {
    std::string component = "bioscara_arm";
    lifecycle_msgs::msg::State current_state = hardware_states_.at(component).state;
    lifecycle_msgs::msg::State target_state = target_state_from_current(current_state);

    set_hardware_component_state(component, target_state);

    // TODO: activate vjtc if all joints are homed and activation will succeed.
   set_controller_state("velocity_joint_trajectory_controller", target_state);
    
    
  }

  void BioscaraPanel::gripper_en_btn_cb(void)
  {
    std::string component = "bioscara_gripper_128";
    lifecycle_msgs::msg::State current_state = hardware_states_.at(component).state;
    lifecycle_msgs::msg::State target_state = target_state_from_current(current_state);

    set_hardware_component_state(component, target_state);

    set_controller_state("gripper_controller", target_state);

  }

  void BioscaraPanel::vjtc_ctrl_en_btn_cb(void)
  {
    std::string controller = "velocity_joint_trajectory_controller";
    lifecycle_msgs::msg::State current_state = controller_states_.at(controller).state;
    lifecycle_msgs::msg::State target_state = target_state_from_current(current_state);

    set_controller_state(controller, target_state);

  }

  void BioscaraPanel::homing_ctrl_en_btn_cb(void)
  {
    std::string controller = "homing_controller";
    lifecycle_msgs::msg::State current_state = controller_states_.at(controller).state;
    lifecycle_msgs::msg::State target_state = target_state_from_current(current_state);

    set_controller_state(controller, target_state);
  }

  void BioscaraPanel::gripper_ctrl_en_btn_cb(void)
  {
    std::string controller = "gripper_controller";
    lifecycle_msgs::msg::State current_state = controller_states_.at(controller).state;
    lifecycle_msgs::msg::State target_state = target_state_from_current(current_state);

    set_controller_state(controller, target_state);
  }

  void BioscaraPanel::dynamic_joint_state_msg_to_map(const DynamicJointState &dynamic_joint_state_in,
                                                     std::unordered_map<std::string, InterfaceValue> &map_out)
  {
    for (size_t i = 0; i < dynamic_joint_state_in.joint_names.size(); i++)
    {
      map_out[dynamic_joint_state_in.joint_names[i]] = dynamic_joint_state_in.interface_values[i];
    }
  }

  void BioscaraPanel::named_lcs_msg_to_map(const std::vector<NamedLifecycleState> &named_lcs_in,
                                           std::unordered_map<std::string, NamedLifecycleState> &map_out)
  {
    for (const auto &lcs : named_lcs_in)
    {
      map_out[lcs.name] = lcs;
    }
  }

  void BioscaraPanel::print_cm_map(const std::string prefix,
                                   const std::unordered_map<std::string, NamedLifecycleState> &map_in)
  {
    std::string states = "";
    for (const auto &[name, state] : map_in)
    {
      states += (name + ":\t" + state.state.label + " (" + std::to_string(state.state.id) + ")\n");
    }
    RCLCPP_DEBUG(node_->get_logger(),
                 "%s\n%s", prefix.c_str(), states.c_str());
  }

  void BioscaraPanel::prepopulate_joint_state_map(std::unordered_map<std::string, InterfaceValue> &state_map,
                                                  std::vector<std::string> augment_vec)
  {
    for (auto &key : augment_vec)
    {
      InterfaceValue dummy;
      dummy.interface_names = {"home", "velocity", "position"};
      dummy.values = {0.0, 0.0, 0.0};
      state_map[key] = dummy;
    }
  }

  void BioscaraPanel::prepopulate_state_map(std::unordered_map<std::string, NamedLifecycleState> &state_map,
                                            std::vector<std::string> augment_vec)
  {
    for (auto &key : augment_vec)
    {
      NamedLifecycleState state;
      state.name = key;
      state_map[key] = state;
    }
  }

  void BioscaraPanel::update_homing_grp_state(void)
  {
    bool enable = false;
    std::string hint = "Enable the homing controller to start homing.";
    switch (controller_states_.at("homing_controller").state.id)
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      enable = true;
      hint = "";
      break;

    default:
      break;
    }
    ui_->homing_grp->setEnabled(enable);
    ui_->homing_state_hint->setText(QString(hint.c_str()));
  }

  void BioscaraPanel::update_homing_state_labels(void)
  {
    set_homing_state_label(ui_->j1_hm_state_label, joint_states_.at("j1"));
    set_homing_state_label(ui_->j2_hm_state_label, joint_states_.at("j2"));
    set_homing_state_label(ui_->j3_hm_state_label, joint_states_.at("j3"));
    set_homing_state_label(ui_->j4_hm_state_label, joint_states_.at("j4"));
  }

  void BioscaraPanel::set_homing_state_label(QLabel *label, const InterfaceValue &state)
  {
    bool homed = false;

    // TODO: this is not very efficient to find the state interface. But works for the small number.
    for (size_t i = 0; i < state.interface_names.size(); i++)
    {
      if (state.interface_names[i] == "home")
      {
        homed = state.values[i] > 0.0;
        break;
      }
    }
    if (homed)
    {
      label->setText(QString("Ready"));
      label->setStyleSheet("QLabel { color : green; font: bold }");
    }
    else
    {
      label->setText(QString("Not Homed"));
      label->setStyleSheet("QLabel { color : red; font: bold }");
    }
  }

  void BioscaraPanel::update_state_labels_and_btns(void)
  {
    update_state_label_and_btn(controller_states_, ui_->vjtc_ctrl_state_label,
                               ui_->vjtc_ctrl_en_btn, "velocity_joint_trajectory_controller");

    update_state_label_and_btn(controller_states_, ui_->homing_ctrl_state_label,
                               ui_->homing_ctrl_en_btn, "homing_controller");

    update_state_label_and_btn(controller_states_, ui_->gripper_ctrl_state_label,
                               ui_->gripper_ctrl_en_btn, "gripper_controller");

    update_state_label_and_btn(hardware_states_, ui_->arm_hardware_state_label,
                               ui_->arm_en_btn, "bioscara_arm");

    update_state_label_and_btn(hardware_states_, ui_->gripper_hardware_state_label,
                               ui_->gripper_en_btn, "bioscara_gripper_128");
  }

  void BioscaraPanel::update_state_label_and_btn(
      const std::unordered_map<std::string, NamedLifecycleState> &state_map,
      QLabel *state_label,
      QPushButton *en_button,
      const std::string &state_key)
  {

    set_state_label(state_label, state_map.at(state_key));
    set_en_btn(en_button, state_map.at(state_key));
  }

  void BioscaraPanel::set_state_label(QLabel *label, const NamedLifecycleState &state)
  {
    label->setText(QString(state.state.label.c_str()));
    switch (state.state.id)
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      label->setStyleSheet("QLabel { color : green; font: bold }");
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      label->setStyleSheet("QLabel { font: bold }");
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      label->setStyleSheet("QLabel { blue; font: bold }");
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      label->setStyleSheet("QLabel { color : red; font: bold }");
      break;

    default:
      label->setStyleSheet("QLabel {font: bold }");
      break;
    }
  }

  void BioscaraPanel::set_en_btn(QPushButton *button, const NamedLifecycleState &state)
  {
    std::string text;
    bool enable = true;
    switch (state.state.id)
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      button->setStyleSheet("QPushButton { color : red}");
      text = "Deactivate";
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      button->setStyleSheet("QPushButton { color : green}");
      text = "Activate";
      enable = check_activation_conditions(state.name);
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      button->setStyleSheet("QPushButton { color : orange}");
      text = "Configure";
      break;

    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:

    default:
      text = "-";
      enable = false;
      break;
    }

    button->setText(QString(text.c_str()));
    button->setEnabled(enable);
  }

  bool BioscaraPanel::check_activation_conditions(const std::string component_key)
  {
    if (component_key == "velocity_joint_trajectory_controller")
    {
      if (hardware_states_.at("bioscara_arm").state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return false;
      }
      if (controller_states_.at("homing_controller").state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return false;
      }
    }
    if (component_key == "homing_controller")
    {
      if (hardware_states_.at("bioscara_arm").state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return false;
      }
      if (controller_states_.at("velocity_joint_trajectory_controller").state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return false;
      }
    }
    if (component_key == "gripper_controller")
    {
      if (hardware_states_.at("bioscara_gripper_128").state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return false;
      }
    }
    return true;
  }

} // namespace bioscara_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(bioscara_rviz_plugin::BioscaraPanel, rviz_common::Panel)
