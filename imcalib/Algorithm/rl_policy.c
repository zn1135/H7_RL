#include "rl_policy.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "mono_ns.h"
#include "networkzn1.h"
#include "networkzn1_data.h"

#if (AI_NETWORKZN1_IN_NUM != 2) || (AI_NETWORKZN1_IN_1_SIZE != RL_OBS_SIZE) || (AI_NETWORKZN1_IN_2_SIZE != RL_OBS_HISTORY_SIZE)
#error "networkzn1 input mismatch"
#endif
#if (AI_NETWORKZN1_OUT_NUM != 2) || (AI_NETWORKZN1_OUT_1_SIZE != RL_ACTION_SIZE) || (AI_NETWORKZN1_OUT_2_SIZE != RL_LATENT_SIZE)
#error "networkzn1 output mismatch"
#endif

static rl_network_t rl_network[RL_MODEL_COUNT];

AI_ALIGNED(4) static ai_u8 standup_activations[AI_NETWORKZN1_DATA_ACTIVATION_1_SIZE];

/* 清空动作 */
static void RL_Policy_Clear_Action(float action[RL_ACTION_SIZE])
{
    if (action != NULL)
    {
        memset(action, 0, RL_ACTION_SIZE * sizeof(float));
    }
}

/* 检查数组 */
static uint8_t RL_Policy_Array_Valid(const float *data, uint32_t count)
{
    uint32_t i;

    if (data == NULL)
    {
        return 0u;
    }
    for (i = 0u; i < count; i++)
    {
        if (!isfinite(data[i]))
        {
            return 0u;
        }
    }
    return 1u;
}

/* 初始化网络: 只有 networkzn1 */
static uint8_t RL_Policy_Init_Network(rl_model_t model)
{
    rl_network_t *network;
    ai_error error;
    ai_u16 input_count;
    ai_u16 output_count;
    const ai_handle activations[] = {AI_HANDLE_PTR(standup_activations)};

    if (model != RL_MODEL_STANDUP)
    {
        return 0u;
    }
    network = &rl_network[model];
    if (network->ready)
    {
        return 1u;
    }

    input_count = 0u;
    output_count = 0u;
    error = ai_networkzn1_create_and_init(&network->network, activations, NULL);
    if (error.type == AI_ERROR_NONE)
    {
        network->inputs = ai_networkzn1_inputs_get(network->network, &input_count);
        network->outputs = ai_networkzn1_outputs_get(network->network, &output_count);
        network->ready = (uint8_t)(network->inputs != NULL && network->outputs != NULL
            && input_count == AI_NETWORKZN1_IN_NUM && output_count == AI_NETWORKZN1_OUT_NUM);
    }

    if (!network->ready)
    {
        network->network = AI_HANDLE_NULL;
        network->inputs = NULL;
        network->outputs = NULL;
    }
    return network->ready;
}

/* 重置策略 */
void RL_Policy_Reset(rl_policy_t *policy)
{
    if (policy == NULL)
    {
        return;
    }
    memset(policy, 0, sizeof(*policy));
    policy->selected_model = RL_MODEL_STANDUP;
}

/* 初始化策略 */
uint8_t RL_Policy_Init(rl_policy_t *policy)
{
    uint32_t i;

    if (policy == NULL)
    {
        return 0u;
    }
    __HAL_RCC_CRC_CLK_ENABLE();

    policy->ready = 1u;
    for (i = 0u; i < RL_MODEL_COUNT; i++)
    {
        policy->model_ready[i] = RL_Policy_Init_Network((rl_model_t)i);
        if (!policy->model_ready[i])
        {
            policy->ready = 0u;
        }
    }
    return policy->ready;
}

/* 选择模型 */
uint8_t RL_Policy_Select(rl_policy_t *policy, rl_model_t model)
{
    if (policy == NULL || model >= RL_MODEL_COUNT)
    {
        return 0u;
    }
    policy->selected_model = model;
    return 1u;
}

/* 执行推理: obs + 历史 → 动作 (输出 0) + latent (输出 1); 记耗时与成败 */
uint8_t RL_Policy_Run(rl_policy_t *policy,
                      const rl_observation_state_t *observation,
                      float action[RL_ACTION_SIZE])
{
    rl_network_t *network;
    ai_float *obs_input;
    ai_float *history_input;
    ai_float *action_output;
    ai_float *latent_output;
    ai_i32 batches;
    uint64_t t0;

    RL_Policy_Clear_Action(action);
    if (policy == NULL || observation == NULL || !policy->ready
        || !observation->valid || !observation->history_ready
        || policy->selected_model >= RL_MODEL_COUNT
        || !RL_Policy_Array_Valid(observation->obs, RL_OBS_SIZE)
        || !RL_Policy_Array_Valid(observation->history, RL_OBS_HISTORY_SIZE))
    {
        return 0u;
    }

    network = &rl_network[policy->selected_model];
    if (!network->ready)
    {
        return 0u;
    }

    obs_input = AI_BUFFER_DATA(&network->inputs[0], ai_float);
    history_input = AI_BUFFER_DATA(&network->inputs[1], ai_float);
    action_output = AI_BUFFER_DATA(&network->outputs[0], ai_float);
    latent_output = AI_BUFFER_DATA(&network->outputs[1], ai_float);
    if (obs_input == NULL || history_input == NULL
        || action_output == NULL || latent_output == NULL)
    {
        return 0u;
    }

    memcpy(obs_input, observation->obs, RL_OBS_SIZE * sizeof(float));
    memcpy(history_input, observation->history, RL_OBS_HISTORY_SIZE * sizeof(float));
    t0 = Mono_Ns_Get();
    batches = ai_networkzn1_run(network->network, network->inputs, network->outputs);
    policy->run_us = (uint32_t)((Mono_Ns_Get() - t0) / 1000u);
    if (batches != 1)
    {
        (void)ai_networkzn1_get_error(network->network);
        policy->run_fail++;
        return 0u;
    }

    memcpy(action, action_output, RL_ACTION_SIZE * sizeof(float));
    memcpy(policy->latent, latent_output, RL_LATENT_SIZE * sizeof(float));
    if (!RL_Policy_Array_Valid(action, RL_ACTION_SIZE))
    {
        RL_Policy_Clear_Action(action);
        policy->run_fail++;
        return 0u;
    }
    policy->run_ok++;
    return 1u;
}
