// Minimal persistent chat adapter for the board's existing RKLLM 1.3.1 SDK.
// Build against its existing header/library; never replace system runtimes.
#include <cstddef>
#include "rkllm.h"
#include <iostream>
#include <string>
#include <cstring>

static std::string answer;
static bool failed=false;
static int result_callback(RKLLMResult* result,void*,LLMCallState state) {
    if(state==RKLLM_RUN_ERROR) failed=true;
    if(state==RKLLM_RUN_NORMAL && result && result->text) {
        answer+=result->text;
        if(answer.size()>4096) return 1;
    }
    return 0;
}
static std::string json_string(const std::string& s) {
    std::string out="\"";
    const char* hex="0123456789abcdef";
    for(unsigned char c:s) {
        if(c=='"' || c=='\\') {out+='\\';out+=c;}
        else if(c<32) {out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}
        else out+=c;
    }
    return out+"\"";
}
int main(int argc,char** argv) {
    if(argc!=2) return 2;
    LLMHandle handle=nullptr;
    RKLLMParam param=rkllm_createDefaultParam();
    param.model_path=argv[1];param.max_new_tokens=192;param.max_context_len=2048;
    param.top_k=20;param.top_p=.9;param.temperature=.7;param.repeat_penalty=1.1;
    param.skip_special_token=true;param.extend_param.base_domain_id=0;param.extend_param.embed_flash=1;
    RKLLMCallback callbacks={};callbacks.result_callback=result_callback;
    if(rkllm_init(&handle,&param,&callbacks)!=0) return 3;
    std::cout<<"\nFALCONS_CHAT_READY\n"<<std::flush;
    std::string text;
    while(std::getline(std::cin,text)) {
        if(text.empty() || text.size()>4096) return 4;
        // History is supplied explicitly by Python, so cancellation cannot retain
        // a half-finished turn in an invisible model-side conversation.
        RKLLMInput input={};input.input_type=RKLLM_INPUT_PROMPT;
        input.role="user";input.enable_thinking=false;input.prompt_input=const_cast<char*>(text.c_str());
        RKLLMInferParam infer={};infer.mode=RKLLM_INFER_GENERATE;infer.keep_history=0;
        answer.clear();failed=false;
        int code=rkllm_run(handle,&input,&infer,nullptr);
        std::cout<<"\nFALCONS_CHAT {\"ok\":"<<((code==0&&!failed)?"true":"false")
                 <<",\"text\":"<<json_string(answer)<<"}\n"<<std::flush;
    }
    rkllm_destroy(handle);
    return 0;
}
