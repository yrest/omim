#include "Framework.hpp"

#include "../../../../../search/result.hpp"

#include "../../../../../map/measurement_utils.hpp"

#include "../../../../../base/thread.hpp"

#include "../core/jni_helper.hpp"


class SearchAdapter
{
  /// @name Results holder. Store last valid results from search threads (m_storeID)
  /// and current result to show in GUI (m_ID).
  //@{
  search::Results m_storeResults, m_results;
  int m_storeID, m_ID;
  //@}

  threads::Mutex m_updateMutex;

  /// Last saved activity to run update UI.
  jobject m_activity;

  // This function may be called several times for one queryID.
  // In that case we should increment m_storeID to distinguish different results.
  // Main queryID is incremented by 5-step to leave space for middle queries.
  // This constant should be equal with SearchActivity.QUERY_STEP;
  static int const QUERY_STEP = 5;

  void OnResults(search::Results const & res, int queryID)
  {
    if (s_pInstance == 0)
    {
      // In case when activity is destroyed, but search thread passed any results.
      return;
    }

    threads::MutexGuard guard(m_updateMutex);

    // store current results
    m_storeResults = res;

    if (m_storeID >= queryID && m_storeID < queryID + QUERY_STEP)
    {
      ++m_storeID;
      // not more than QUERY_STEP results per query
      ASSERT_LESS ( m_storeID, queryID + QUERY_STEP, () );
    }
    else
    {
      ASSERT_LESS ( m_storeID, queryID, () );
      m_storeID = queryID;
    }

    // get new environment pointer here because of different thread
    JNIEnv * env = jni::GetEnv();

    // post message to update ListView in UI thread
    jmethodID const id = jni::GetJavaMethodID(env, m_activity, "updateData", "(II)V");
    env->CallVoidMethod(m_activity, id,
                          static_cast<jint>(m_storeResults.GetCount()),
                          static_cast<jint>(m_storeID));
  }

  bool AcquireShowResults(int resultID)
  {
    if (resultID != m_ID)
    {
      {
        // Grab last results.
        threads::MutexGuard guard(m_updateMutex);
        m_results.Swap(m_storeResults);
        m_ID = m_storeID;
      }

      if (resultID != m_ID)
      {
        // It happens only when better results came faster than GUI.
        // It is a rare case, skip this query.
        ASSERT_GREATER ( m_ID, resultID, () );
        return false;
      }
    }

    return true;
  }

  bool CheckPosition(int position) const
  {
    int const count = static_cast<int>(m_results.GetCount());

    // for safety reasons do actual check always
    ASSERT_LESS ( position, count, () );
    return (position < count);
  }

  SearchAdapter(jobject activity) : m_ID(0), m_storeID(0), m_activity(activity)
  {
  }

  static SearchAdapter * s_pInstance;

public:
  /// @name Instance lifetime functions.
  /// TODO May be we should increment/deincrement global reference for m_activity
  //@{
  static void CreateInstance(jobject activity)
  {
    ASSERT ( s_pInstance == 0, () );
    s_pInstance = new SearchAdapter(activity);
  }

  static void DestroyInstance()
  {
    ASSERT ( s_pInstance, () );
    delete s_pInstance;
    s_pInstance = 0;
  }

  static SearchAdapter & Instance()
  {
    ASSERT ( s_pInstance, () );
    return *s_pInstance;
  }
  //@}

  void RunSearch(JNIEnv * env, search::SearchParams & params, int queryID)
  {
    params.m_callback = bind(&SearchAdapter::OnResults, this, _1, queryID);

    g_framework->Search(params);
  }

  void ShowItem(int position)
  {
    if (CheckPosition(position))
      g_framework->ShowSearchResult(m_results.GetResult(position));
  }

  search::Result const * GetResult(int position, int resultID)
  {
    if (AcquireShowResults(resultID) && CheckPosition(position))
      return &(m_results.GetResult(position));
    return 0;
  }
};

SearchAdapter * SearchAdapter::s_pInstance = 0;

extern "C"
{

JNIEXPORT void JNICALL
Java_com_mapswithme_maps_SearchActivity_nativeInitSearch(JNIEnv * env, jobject thiz)
{
  SearchAdapter::CreateInstance(thiz);
}

JNIEXPORT void JNICALL
Java_com_mapswithme_maps_SearchActivity_nativeFinishSearch(JNIEnv * env, jobject thiz)
{
  SearchAdapter::DestroyInstance();
}

JNIEXPORT void JNICALL
Java_com_mapswithme_maps_SearchActivity_nativeRunSearch(
    JNIEnv * env, jobject thiz,
    jstring s, jstring lang, jdouble lat, jdouble lon, jint mode, jint queryID)
{
  search::SearchParams params;
  params.m_query = jni::ToNativeString(env, s);
  params.SetInputLanguage(jni::ToNativeString(env, lang));
  if (mode != 0)
    params.SetPosition(lat, lon);

  SearchAdapter::Instance().RunSearch(env, params, queryID);
}

JNIEXPORT void JNICALL
Java_com_mapswithme_maps_SearchActivity_nativeShowItem(JNIEnv * env, jobject thiz, jint position)
{
  SearchAdapter::Instance().ShowItem(position);
}

JNIEXPORT jobject JNICALL
Java_com_mapswithme_maps_SearchActivity_nativeGetResult(
    JNIEnv * env, jobject thiz, jint position, jint queryID)
{
  search::Result const * res = SearchAdapter::Instance().GetResult(position, queryID);
  if (res == 0) return 0;

  jclass klass = env->FindClass("com/mapswithme/maps/SearchActivity$SearchAdapter$SearchResult");
  ASSERT ( klass, () );

  if (res->GetResultType() == search::Result::RESULT_FEATURE)
  {
    jmethodID methodID = env->GetMethodID(
        klass, "<init>",
        "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");
    ASSERT ( methodID, () );

    string distance;
    double const d = res->GetDistanceFromCenter();
    if (d >= 0.0)
      CHECK ( MeasurementUtils::FormatDistance(d, distance), () );

    return env->NewObject(klass, methodID,
                          jni::ToJavaString(env, res->GetString()),
                          jni::ToJavaString(env, res->GetRegionString()),
                          jni::ToJavaString(env, res->GetFeatureType()),
                          jni::ToJavaString(env, distance.c_str()),
                          jni::ToJavaString(env, res->GetRegionFlag()));
  }
  else
  {
    jmethodID methodID = env->GetMethodID(klass, "<init>", "(Ljava/lang/String;)V");
    ASSERT ( methodID, () );

    return env->NewObject(klass, methodID, jni::ToJavaString(env, res->GetSuggestionString()));
  }
}

}
