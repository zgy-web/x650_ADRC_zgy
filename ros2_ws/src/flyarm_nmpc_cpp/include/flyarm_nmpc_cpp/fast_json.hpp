#pragma once
// Lossless auxiliary-thread JSON encoding. No precision truncation, compression,
// dropped arrays or reduced diagnostics: only scalar formatting is replaced.
#include <json/json.h>
#include <charconv>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace flyarm_nmpc_cpp {
inline void json_quote(std::string& out,const std::string& value) {
  constexpr char hex[]="0123456789abcdef";out.push_back('"');
  for(const unsigned char c:value){switch(c){
    case '"':out+="\\\"";break;case '\\':out+="\\\\";break;
    case '\b':out+="\\b";break;case '\f':out+="\\f";break;
    case '\n':out+="\\n";break;case '\r':out+="\\r";break;case '\t':out+="\\t";break;
    default:if(c<0x20){out+="\\u00";out.push_back(hex[c>>4]);out.push_back(hex[c&15]);}else out.push_back(char(c));
  }}out.push_back('"');
}
template<class Number> inline void json_integer(std::string& out,Number value) {
  char buffer[32];const auto converted=std::to_chars(buffer,buffer+sizeof(buffer),value);
  if(converted.ec!=std::errc{})throw std::runtime_error("JSON integer conversion failed");
  out.append(buffer,converted.ptr);
}
inline void json_append(std::string& out,const Json::Value& value,unsigned depth=0) {
  if(depth>128)throw std::runtime_error("Diagnostic JSON nesting exceeds bound");
  switch(value.type()){
    case Json::nullValue:out+="null";break;
    case Json::intValue:json_integer(out,value.asInt64());break;
    case Json::uintValue:json_integer(out,value.asUInt64());break;
    case Json::realValue:{
      const double v=value.asDouble();if(!std::isfinite(v))throw std::runtime_error("Nonfinite diagnostic JSON real");
      char buffer[128];const auto converted=std::to_chars(buffer,buffer+sizeof(buffer),v,std::chars_format::general,std::numeric_limits<double>::max_digits10);
      if(converted.ec!=std::errc{})throw std::runtime_error("JSON real conversion failed");
      bool real_token=false;for(const char* p=buffer;p!=converted.ptr;++p)real_token=real_token||*p=='.'||*p=='e'||*p=='E';
      out.append(buffer,converted.ptr);if(!real_token)out+=".0"; // preserve real type and signed zero
      break;}
    case Json::stringValue:json_quote(out,value.asString());break;
    case Json::booleanValue:out+=value.asBool()?"true":"false";break;
    case Json::arrayValue:{out.push_back('[');bool first=true;for(const auto& x:value){if(!first)out.push_back(',');first=false;json_append(out,x,depth+1);}out.push_back(']');break;}
    case Json::objectValue:{out.push_back('{');bool first=true;for(auto i=value.begin();i!=value.end();++i){if(!first)out.push_back(',');first=false;json_quote(out,i.name());out.push_back(':');json_append(out,*i,depth+1);}out.push_back('}');break;}
    default:throw std::runtime_error("Unknown JSON value type");
  }
}
inline std::string encode_json(const Json::Value& value) {
  std::string out;out.reserve(1<<20);json_append(out,value);out.push_back('\n');return out;
}
}  // namespace flyarm_nmpc_cpp
