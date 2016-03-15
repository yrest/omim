#include "editor/osm_auth.hpp"

#include "coding/url_encode.hpp"

#include "base/assert.hpp"
#include "base/logging.hpp"
#include "base/string_utils.hpp"

#include "std/iostream.hpp"
#include "std/map.hpp"

#include "private.h"

#include "3party/liboauthcpp/include/liboauthcpp/liboauthcpp.h"
#include "3party/Alohalytics/src/http_client.h"

using alohalytics::HTTPClientPlatformWrapper;

namespace osm
{
constexpr char const * kApiVersion = "/api/0.6";
constexpr char const * kFacebookCallbackPart = "/auth/facebook_access_token/callback?access_token=";
constexpr char const * kGoogleCallbackPart = "/auth/google_oauth2_access_token/callback?access_token=";
constexpr char const * kFacebookOAuthPart = "/auth/facebook?referer=%2Foauth%2Fauthorize%3Foauth_token%3D";
constexpr char const * kGoogleOAuthPart = "/auth/google?referer=%2Foauth%2Fauthorize%3Foauth_token%3D";

namespace
{

string FindAuthenticityToken(string const & body)
{
  auto pos = body.find("name=\"authenticity_token\"");
  if (pos == string::npos)
    return string();
  string const kValue = "value=\"";
  auto start = body.find(kValue, pos);
  if (start == string::npos)
    return string();
  start += kValue.length();
  auto const end = body.find("\"", start);
  return end == string::npos ? string() : body.substr(start, end - start);
}

string BuildPostRequest(map<string, string> const & params)
{
  string result;
  for (auto it = params.begin(); it != params.end(); ++it)
  {
    if (it != params.begin())
      result += "&";
    result += it->first + "=" + UrlEncode(it->second);
  }
  return result;
}

// Trying to determine whether it's a login page.
bool IsLoggedIn(string const & contents)
{
  return contents.find("<form id=\"login_form\"") == string::npos;
}

// TODO(AlexZ): DebugPrint doesn't detect this overload. Fix it.
string DP(alohalytics::HTTPClientPlatformWrapper const & request)
{
  string str = "HTTP " + strings::to_string(request.error_code()) + " url [" + request.url_requested() + "]";
  if (request.was_redirected())
    str += " was redirected to [" + request.url_received() + "]";
  if (!request.server_response().empty())
    str += " response: " + request.server_response();
  return str;
}

}  // namespace

// static
bool OsmOAuth::IsValid(TKeySecret const & ks) noexcept
{
  return !(ks.first.empty() || ks.second.empty());
}
// static
bool OsmOAuth::IsValid(TUrlRequestToken const & urt) noexcept
{
  return !(urt.first.empty() || urt.second.first.empty() || urt.second.second.empty());
}

OsmOAuth::OsmOAuth(string const & consumerKey, string const & consumerSecret,
                   string const & baseUrl, string const & apiUrl) noexcept
  : m_consumerKeySecret(consumerKey, consumerSecret), m_baseUrl(baseUrl), m_apiUrl(apiUrl)
{
}
// static
OsmOAuth OsmOAuth::ServerAuth() noexcept
{
#ifdef DEBUG
  return IZServerAuth();
#else
  return ProductionServerAuth();
#endif
}
// static
OsmOAuth OsmOAuth::ServerAuth(TKeySecret const & userKeySecret) noexcept
{
  OsmOAuth auth = ServerAuth();
  auth.SetKeySecret(userKeySecret);
  return auth;
}
// static
OsmOAuth OsmOAuth::IZServerAuth() noexcept
{
  constexpr char const * kIZTestServer = "http://188.166.112.124:3000";
  constexpr char const * kIZConsumerKey = "QqwiALkYZ4Jd19lo1dtoPhcwGQUqMCMeVGIQ8Ahb";
  constexpr char const * kIZConsumerSecret = "wi9HZKFoNYS06Yad5s4J0bfFo2hClMlH7pXaXWS3";
  return OsmOAuth(kIZConsumerKey, kIZConsumerSecret, kIZTestServer, kIZTestServer);
}
// static
OsmOAuth OsmOAuth::DevServerAuth() noexcept
{
  constexpr char const * kOsmDevServer = "http://master.apis.dev.openstreetmap.org";
  constexpr char const * kOsmDevConsumerKey = "eRtN6yKZZf34oVyBnyaVbsWtHIIeptLArQKdTwN3";
  constexpr char const * kOsmDevConsumerSecret = "lC124mtm2VqvKJjSh35qBpKfrkeIjpKuGe38Hd1H";
  return OsmOAuth(kOsmDevConsumerKey, kOsmDevConsumerSecret, kOsmDevServer, kOsmDevServer);
}
// static
OsmOAuth OsmOAuth::ProductionServerAuth() noexcept
{
  constexpr char const * kOsmMainSiteURL = "https://www.openstreetmap.org";
  constexpr char const * kOsmApiURL = "https://api.openstreetmap.org";
  return OsmOAuth(OSM_CONSUMER_KEY, OSM_CONSUMER_SECRET, kOsmMainSiteURL, kOsmApiURL);
}

void OsmOAuth::SetKeySecret(TKeySecret const & keySecret) noexcept { m_tokenKeySecret = keySecret; }

TKeySecret const & OsmOAuth::GetKeySecret() const noexcept { return m_tokenKeySecret; }

bool OsmOAuth::IsAuthorized() const noexcept{ return IsValid(m_tokenKeySecret); }

// Opens a login page and extract a cookie and a secret token.
OsmOAuth::SessionID OsmOAuth::FetchSessionId(string const & subUrl) const
{
  string const url = m_baseUrl + subUrl + "?cookie_test=true";
  HTTPClientPlatformWrapper request(url);
  if (!request.RunHTTPRequest())
    MYTHROW(NetworkError, ("FetchSessionId Network error while connecting to", url));
  if (request.was_redirected())
    MYTHROW(UnexpectedRedirect, ("Redirected to", request.url_received(), "from", url));
  if (request.error_code() != HTTP::OK)
    MYTHROW(FetchSessionIdError, (DP(request)));

  SessionID const sid = { request.combined_cookies(), FindAuthenticityToken(request.server_response()) };
  if (sid.m_cookies.empty() || sid.m_token.empty())
    MYTHROW(FetchSessionIdError, ("Cookies and/or token are empty for request", DP(request)));
  return sid;
}

void OsmOAuth::LogoutUser(SessionID const & sid) const
{
  HTTPClientPlatformWrapper request(m_baseUrl + "/logout");
  request.set_cookies(sid.m_cookies);
  if (!request.RunHTTPRequest())
    MYTHROW(NetworkError, ("LogoutUser Network error while connecting to", request.url_requested()));
  if (request.error_code() != HTTP::OK)
    MYTHROW(LogoutUserError, (DP(request)));
}

bool OsmOAuth::LoginUserPassword(string const & login, string const & password, SessionID const & sid) const
{
  map<string, string> const params =
  {
    {"username", login},
    {"password", password},
    {"referer", "/"},
    {"commit", "Login"},
    {"authenticity_token", sid.m_token}
  };
  HTTPClientPlatformWrapper request(m_baseUrl + "/login");
  request.set_body_data(BuildPostRequest(params), "application/x-www-form-urlencoded");
  request.set_cookies(sid.m_cookies);
  if (!request.RunHTTPRequest())
    MYTHROW(NetworkError, ("LoginUserPassword Network error while connecting to", request.url_requested()));
  if (request.error_code() != HTTP::OK)
    MYTHROW(LoginUserPasswordServerError, (DP(request)));

  // Not redirected page is a 100% signal that login and/or password are invalid.
  if (!request.was_redirected())
    return false;
  // Parse redirected page contents to make sure that it's not some router in-between.
  return IsLoggedIn(request.server_response());
}

bool OsmOAuth::LoginSocial(string const & callbackPart, string const & socialToken, SessionID const & sid) const
{
  string const url = m_baseUrl + callbackPart + socialToken;
  HTTPClientPlatformWrapper request(url);
  request.set_cookies(sid.m_cookies);
  if (!request.RunHTTPRequest())
    MYTHROW(NetworkError, ("LoginSocial Network error while connecting to", request.url_requested()));
  if (request.error_code() != HTTP::OK)
    MYTHROW(LoginSocialServerError, (DP(request)));

  // Not redirected page is a 100% signal that social login has failed.
  if (!request.was_redirected())
    return false;
  // Parse redirected page contents to make sure that it's not some router in-between.
  return IsLoggedIn(request.server_response());
}

// Fakes a buttons press to automatically accept requested permissions.
string OsmOAuth::SendAuthRequest(string const & requestTokenKey, SessionID const & sid) const
{
  map<string, string> const params =
  {
    {"oauth_token", requestTokenKey},
    {"oauth_callback", ""},
    {"authenticity_token", sid.m_token},
    {"allow_read_prefs", "yes"},
    {"allow_write_api", "yes"},
    {"allow_write_gpx", "yes"},
    {"allow_write_notes", "yes"},
    {"commit", "Save changes"}
  };
  HTTPClientPlatformWrapper request(m_baseUrl + "/oauth/authorize");
  request.set_body_data(BuildPostRequest(params), "application/x-www-form-urlencoded");
  request.set_cookies(sid.m_cookies);
  if (!request.RunHTTPRequest())
    MYTHROW(NetworkError, ("SendAuthRequest Network error while connecting to", request.url_requested()));

  string const callbackURL = request.url_received();
  string const vKey = "oauth_verifier=";
  auto const pos = callbackURL.find(vKey);
  if (pos == string::npos)
    MYTHROW(SendAuthRequestError, ("oauth_verifier is not found", DP(request)));

  auto const end = callbackURL.find("&", pos);
  return callbackURL.substr(pos + vKey.length(), end == string::npos ? end : end - pos - vKey.length());
}

TRequestToken OsmOAuth::FetchRequestToken() const
{
  OAuth::Consumer const consumer(m_consumerKeySecret.first, m_consumerKeySecret.second);
  OAuth::Client oauth(&consumer);
  string const requestTokenUrl = m_baseUrl + "/oauth/request_token";
  string const requestTokenQuery = oauth.getURLQueryString(OAuth::Http::Get, requestTokenUrl + "?oauth_callback=oob");
  HTTPClientPlatformWrapper request(requestTokenUrl + "?" + requestTokenQuery);
  if (!request.RunHTTPRequest())
    MYTHROW(NetworkError, ("FetchRequestToken Network error while connecting to", request.url_requested()));
  if (request.error_code() != HTTP::OK)
    MYTHROW(FetchRequestTokenServerError, (DP(request)));
  if (request.was_redirected())
    MYTHROW(UnexpectedRedirect, ("Redirected to", request.url_received(), "from", request.url_requested()));

  // Throws std::runtime_error.
  OAuth::Token const reqToken = OAuth::Token::extract(request.server_response());
  return { reqToken.key(), reqToken.secret() };
}

TKeySecret OsmOAuth::FinishAuthorization(TRequestToken const & requestToken, string const & verifier) const
{
  OAuth::Consumer const consumer(m_consumerKeySecret.first, m_consumerKeySecret.second);
  OAuth::Token const reqToken(requestToken.first, requestToken.second, verifier);
  OAuth::Client oauth(&consumer, &reqToken);
  string const accessTokenUrl = m_baseUrl + "/oauth/access_token";
  string const queryString = oauth.getURLQueryString(OAuth::Http::Get, accessTokenUrl, "", true);
  HTTPClientPlatformWrapper request(accessTokenUrl + "?" + queryString);
  if (!request.RunHTTPRequest())
    MYTHROW(NetworkError, ("FinishAuthorization Network error while connecting to", request.url_requested()));
  if (request.error_code() != HTTP::OK)
    MYTHROW(FinishAuthorizationServerError, (DP(request)));
  if (request.was_redirected())
    MYTHROW(UnexpectedRedirect, ("Redirected to", request.url_received(), "from", request.url_requested()));

  OAuth::KeyValuePairs const responseData = OAuth::ParseKeyValuePairs(request.server_response());
  // Throws std::runtime_error.
  OAuth::Token const accessToken = OAuth::Token::extract(responseData);
  return { accessToken.key(), accessToken.secret() };
}

// Given a web session id, fetches an OAuth access token.
TKeySecret OsmOAuth::FetchAccessToken(SessionID const & sid) const
{
  // Aquire a request token.
  TRequestToken const requestToken = FetchRequestToken();

  // Faking a button press for access rights.
  string const pin = SendAuthRequest(requestToken.first, sid);
  LogoutUser(sid);

  // Got pin, exchange it for the access token.
  return FinishAuthorization(requestToken, pin);
}

bool OsmOAuth::AuthorizePassword(string const & login, string const & password)
{
  SessionID const sid = FetchSessionId();
  if (!LoginUserPassword(login, password, sid))
    return false;
  m_tokenKeySecret = FetchAccessToken(sid);
  return true;
}

bool OsmOAuth::AuthorizeFacebook(string const & facebookToken)
{
  SessionID const sid = FetchSessionId();
  if (!LoginSocial(kFacebookCallbackPart, facebookToken, sid))
    return false;
  m_tokenKeySecret = FetchAccessToken(sid);
  return true;
}

bool OsmOAuth::AuthorizeGoogle(string const & googleToken)
{
  SessionID const sid = FetchSessionId();
  if (!LoginSocial(kGoogleCallbackPart, googleToken, sid))
    return false;
  m_tokenKeySecret = FetchAccessToken(sid);
  return true;
}

OsmOAuth::TUrlRequestToken OsmOAuth::GetFacebookOAuthURL() const
{
  TRequestToken const requestToken = FetchRequestToken();
  string const url = m_baseUrl + kFacebookOAuthPart + requestToken.first;
  return TUrlRequestToken(url, requestToken);
}

OsmOAuth::TUrlRequestToken OsmOAuth::GetGoogleOAuthURL() const
{
  TRequestToken const requestToken = FetchRequestToken();
  string const url = m_baseUrl + kGoogleOAuthPart + requestToken.first;
  return TUrlRequestToken(url, requestToken);
}

bool OsmOAuth::ResetPassword(string const & email) const
{
  string const kForgotPasswordUrlPart = "/user/forgot-password";

  SessionID const sid = FetchSessionId(kForgotPasswordUrlPart);
  map<string, string> const params =
  {
    {"user[email]", email},
    {"authenticity_token", sid.m_token},
    {"commit", "Reset password"}
  };
  HTTPClientPlatformWrapper request(m_baseUrl + kForgotPasswordUrlPart);
  request.set_body_data(BuildPostRequest(params), "application/x-www-form-urlencoded");
  request.set_cookies(sid.m_cookies);

  if (!request.RunHTTPRequest())
    MYTHROW(NetworkError, ("ResetPassword Network error while connecting to", request.url_requested()));
  if (request.error_code() != HTTP::OK)
    MYTHROW(ResetPasswordServerError, (DP(request)));

  if (request.was_redirected() && request.url_received().find(m_baseUrl) != string::npos)
    return true;
  return false;
}

OsmOAuth::Response OsmOAuth::Request(string const & method, string const & httpMethod, string const & body) const
{
  if (!IsValid(m_tokenKeySecret))
    MYTHROW(InvalidKeySecret, ("User token (key and secret) are empty."));

  OAuth::Consumer const consumer(m_consumerKeySecret.first, m_consumerKeySecret.second);
  OAuth::Token const oatoken(m_tokenKeySecret.first, m_tokenKeySecret.second);
  OAuth::Client oauth(&consumer, &oatoken);

  OAuth::Http::RequestType reqType;
  if (httpMethod == "GET")
    reqType = OAuth::Http::Get;
  else if (httpMethod == "POST")
    reqType = OAuth::Http::Post;
  else if (httpMethod == "PUT")
    reqType = OAuth::Http::Put;
  else if (httpMethod == "DELETE")
    reqType = OAuth::Http::Delete;
  else
    MYTHROW(UnsupportedApiRequestMethod, ("Unsupported OSM API request method", httpMethod));

  string url = m_apiUrl + kApiVersion + method;
  string const query = oauth.getURLQueryString(reqType, url);
  auto const qPos = url.find('?');
  if (qPos != string::npos)
    url = url.substr(0, qPos);

  HTTPClientPlatformWrapper request(url + "?" + query);
  if (httpMethod != "GET")
    request.set_body_data(body, "application/xml", httpMethod);
  if (!request.RunHTTPRequest())
    MYTHROW(NetworkError, ("Request Network error while connecting to", url));
  if (request.was_redirected())
    MYTHROW(UnexpectedRedirect, ("Redirected to", request.url_received(), "from", url));

  return Response(request.error_code(), request.server_response());
}

OsmOAuth::Response OsmOAuth::DirectRequest(string const & method, bool api) const
{
  string const url = api ? m_apiUrl + kApiVersion + method : m_baseUrl + method;
  HTTPClientPlatformWrapper request(url);
  if (!request.RunHTTPRequest())
    MYTHROW(NetworkError, ("DirectRequest Network error while connecting to", url));
  if (request.was_redirected())
    MYTHROW(UnexpectedRedirect, ("Redirected to", request.url_received(), "from", url));

  return Response(request.error_code(), request.server_response());
}

string DebugPrint(OsmOAuth::Response const & code)
{
  string r;
  switch (code.first)
  {
  case OsmOAuth::HTTP::OK: r = "OK"; break;
  case OsmOAuth::HTTP::BadXML: r = "BadXML"; break;
  case OsmOAuth::HTTP::BadAuth: r = "BadAuth"; break;
  case OsmOAuth::HTTP::Redacted: r = "Redacted"; break;
  case OsmOAuth::HTTP::NotFound: r = "NotFound"; break;
  case OsmOAuth::HTTP::WrongMethod: r = "WrongMethod"; break;
  case OsmOAuth::HTTP::Conflict: r = "Conflict"; break;
  case OsmOAuth::HTTP::Gone: r = "Gone"; break;
  case OsmOAuth::HTTP::PreconditionFailed: r = "PreconditionFailed"; break;
  case OsmOAuth::HTTP::URITooLong: r = "URITooLong"; break;
  case OsmOAuth::HTTP::TooMuchData: r = "TooMuchData"; break;
  default:
    // No data from server in case of NetworkError.
    if (code.first < 0)
      return "NetworkError " + strings::to_string(code.first);
    r = "HTTP " + strings::to_string(code.first);
  }
  return r + ": " + code.second;
}

}  // namespace osm
