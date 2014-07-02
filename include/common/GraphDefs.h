/*! 
 *  \file    GraphDefs.h
 *  \author  Shashank Pathak
 *  \date    2012
 * 
 */
#ifndef MYGRAPH_H
#define MYGRAPH_H

#include <Configuration.h>
#include <StateAction.h>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/composite_key.hpp>

// include headers that implement a archive in simple text format
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>


#include <boost/tuple/tuple.hpp>

//#include <unordered_map>        // For Q, T and other tables
#include <boost/unordered_map.hpp>
//#include <unordered_map_serialization.h>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

//#include <boost_serialize_unordered_map.hpp>
//#include <UnorderedMapImpl.hpp>
//#include <SerializeTuple.hpp>


using namespace boost;

enum edge_unsafety_t { edge_unsafety };
enum edge_action_t { edge_action };
enum edge_propensity_t { edge_propensity };

namespace boost {
  BOOST_INSTALL_PROPERTY(edge, unsafety);
  BOOST_INSTALL_PROPERTY(edge, action);
  BOOST_INSTALL_PROPERTY(edge, propensity);
}

enum vertex_discreteState_t { vertex_discreteState };
enum vertex_id_t { vertex_id };
namespace boost {
  BOOST_INSTALL_PROPERTY(vertex, discreteState);
  BOOST_INSTALL_PROPERTY(vertex, id);
}

template < typename OutputIterator >
class back_edge_recorder : public default_dfs_visitor
{
public:
  back_edge_recorder(OutputIterator out):m_out(out) { }

  template < typename Edge, typename Graph >
  void back_edge(Edge e, const Graph &g)
  {
    *m_out++ = e;
//      property_map<Graph, edge_unsafety_t>::type unsafeMap
//          = get(edge_unsafety_t(), g);
//      bool unsafe = unSafeMap[e];
      /////std::cout<<"Back edge at: \n";
  }
private:
  OutputIterator m_out;
};

// object generator function
template <PraveenKumar Vasudevan typename OutputIterator >
back_edge_recorder < OutputIterator >
make_back_edge_recorder(OutputIterator out)
{
  return back_edge_recorder < OutputIterator > (out);
}

template < typename Graph, typename BackEdgeVert> void
find_loops(typename graph_traits < Graph >::vertex_descriptor entry,
           const Graph & g,
           BackEdgeVert & backedgeVert)    // A container of sets of vertices
{
  BOOST_CONCEPT_ASSERT(( BidirectionalGraphConcept<Graph> ));
  typedef typename graph_traits < Graph >::edge_descriptor Edge;
  typedef typename graph_traits < Graph >::vertex_descriptor Vertex;
  std::vector < Edge > back_edges;
  std::vector < default_color_type > color_map(num_vertices(g));
  depth_first_visit(g, entry,
                    make_back_edge_recorder(std::back_inserter(back_edges)),
                    make_iterator_property_map(color_map.begin(),
                                               boost::get(vertex_index, g), color_map[0]));

  typedef typename std::vector < Edge >::size_type stype;
  for (stype i = 0; i < back_edges.size(); ++i) {
      //typename Loops::value_type x;
      //loops.push_back(x);
//      fromV = boost::get(vmap, source(back_edges[i], g));
//      toV = boost::get(vmap, target(back_edges[i], g));
      backedgeVert.push_back(back_edges[i]);
            //cout<<"Back edge is from "<<fromV<<" to"<<toV<<"\n";

  }


}

typedef property<edge_unsafety_t, bool> Unsaf;
  typedef property<edge_action_t, int, Unsaf> Acti;
    typedef property<edge_propensity_t,double,Acti> Prope;


typedef boost::adjacency_list<
    vecS, vecS, bidirectionalS,
    // Vertex properties
    boost::property<vertex_id_t, int,
        boost::property<vertex_discreteState_t, int > >,
    // Edge properties
    Prope >
  NewBiDirGraph;

typedef boost::adjacency_list<
    vecS, listS, directedS,
    // Vertex properties
    boost::property<vertex_id_t, int,
        boost::property<vertex_discreteState_t, int > >,
    // Edge properties
    Prope >
  MutableTGraph;



typedef property_map<NewBiDirGraph,
      vertex_discreteState_t>::type VertMap;
typedef property_map<NewBiDirGraph,
      vertex_id_t>::type verIdMap;
typedef graph_traits < NewBiDirGraph >::vertex_descriptor NewVertex;
typedef graph_traits < NewBiDirGraph >::vertex_iterator NewViter;
typedef graph_traits < NewBiDirGraph >::edge_descriptor NewEdge;
typedef graph_traits < NewBiDirGraph >::edge_iterator NewEiter;




typedef property_map<NewBiDirGraph,
        edge_action_t>::type edgActMap;
typedef property_map<NewBiDirGraph,
        edge_unsafety_t>::type edgUnsafMap;
typedef property_map<NewBiDirGraph,
        edge_propensity_t>::type edgPropeMap;

typedef property_map<MutableTGraph,
        vertex_id_t>::type vertexIdType;


typedef boost::graph_traits < NewBiDirGraph >::out_edge_iterator new_out_edge_iter;
typedef boost::graph_traits < NewBiDirGraph >::in_edge_iterator new_in_edge_iter;

typedef boost::graph_traits < MutableTGraph >::out_edge_iterator mut_out_edge_iter;
typedef boost::graph_traits < MutableTGraph >::in_edge_iterator mut_in_edge_iter;

typedef property_map<MutableTGraph,
      vertex_discreteState_t>::type VertMap1;
typedef graph_traits < MutableTGraph >::vertex_descriptor NewVertex1;
typedef graph_traits < MutableTGraph >::vertex_iterator NewViter1;
typedef graph_traits < MutableTGraph >::edge_descriptor NewEdge1;
typedef graph_traits < MutableTGraph >::edge_iterator NewEiter1;

typedef property_map<MutableTGraph,
        vertex_id_t>::type vertexIndexMap1;
typedef property_map<MutableTGraph,
        edge_action_t>::type edgActMap1;
typedef property_map<MutableTGraph,
        edge_unsafety_t>::type edgUnsafMap1;
typedef property_map<MutableTGraph,
        edge_propensity_t>::type edgPropeMap1;

// Markers for deleting
#define DELETE_VERTEX(x) (boost::put(discreteStateMap,x,-1))
#define DELETED_VERTEX(x) (boost::get(discreteStateMap,x) == -1 )
#define DELETE_EDGE(x) (boost::put(actionMap,x,-1))
#define DELETED_EDGE(x) (boost::get(actionMap,x) == -1 )


// Predicate Function for use for filtering in map
template <class ValPropertyMap, class Filtertype>
struct tag_equals_predicate
{
  tag_equals_predicate(const Filtertype& x, ValPropertyMap value)
    : m_val(x), m_value_map(value) { }

  template <class VertexOrEdge>
  bool operator()(const VertexOrEdge& e) const {
    return boost::get(m_val,e) == m_value_map[e];
  }
  Filtertype m_val;
  ValPropertyMap m_value_map;
};
// helper creation function
template <class ValPropertyMap, class Filtertype>
inline tag_equals_predicate<ValPropertyMap,Filtertype>
tag_equals(Filtertype& val, ValPropertyMap value) {
  return tag_equals_predicate<ValPropertyMap,Filtertype>(val, value);
}

template <class Name, class Category>
class myVertexWriter {
public:
    //typedef typename graph_traits < Name >::vertex_descriptor Vertex;
    //typedef typename graph_traits < Name >::size_type Vtype;
    myVertexWriter(Name _name, Category _cat) : name(_name), cat(_cat) {}
     template <class VertexOrEdge>
     void operator()(std::ostream& out, const VertexOrEdge& v) const {


         //LATER:if(tag_equals(boost::get(cat,v),cat))
         switch (boost::get(cat,v))
         {
         case 0:
             out << "[label=\" Init \"]";
             out << "[style=filled] ";
             break;
         case 1:
             out << "[label=\" Unsafe \"]";
             out << "[style=filled] ";
             break;
         case 2:
             out << "[label=\" Safe \"]";
             out << "[style=filled] ";
             break;
         default:
             out << "[label=\"" << boost::get(name,v) << "\"]";
             out << "[style=filled] ";

         }

     }
private:
     Name name;
     Category cat;
};

template <class Name, class Filter, class Filter2>
class myEdgeWriter {
public:

    myEdgeWriter(Name _name,Filter _filter,Filter2 _filter2) : name(_name),filter(_filter),filter2(_filter2) {}
     template <class VertexOrEdge>
     void operator()(std::ostream& out, const VertexOrEdge& v) const {
            out << "[label=\"" << boost::get(name,v) << "\"]";
            if(boost::get(filter2,v) == -1)
                out<<"[color=white]";
            else if(boost::get(filter,v) == true)
            {
               out << "[color=red] ";
            }
            else
            {
               out << "[color=blue] ";
            }

     }
private:
     Name name;
     Filter filter;
     Filter2 filter2;
};

/* Maps for constructing DTMC */

typedef typename boost::tuples::tuple<int,int,double,int,bool> experience;

typedef boost::tuples::tuple<int,int> StateActionTuple;

struct ihash
    : std::unary_function<StateActionTuple, std::size_t>
{
    std::size_t operator()(StateActionTuple const& e) const
    {
        std::size_t seed = 0;
        boost::hash_combine( seed, e.get<0>() );
        boost::hash_combine( seed, e.get<1>() );

        return seed;
    }
};

struct iequal_to
    : std::binary_function<StateActionTuple, StateActionTuple, bool>
{
    bool operator()(StateActionTuple const& x, StateActionTuple const& y) const
    {

        return (x.get<0>()==y.get<0>() &&
                x.get<1>()==y.get<1>());
    }
};

typedef boost::unordered_map< StateActionTuple, double, ihash, iequal_to > StateActionMap;
typedef StateActionMap::iterator QMapIter;


typedef boost::tuples::tuple<int,
                            int,
                            int,
                            bool> TransitTuple;/* <s,a,s',unsafe> and TransitMap stores count of these */
struct ihashT
    : std::unary_function<TransitTuple, std::size_t>
{
    std::size_t operator()(TransitTuple const& e) const
    {
        std::size_t seed = 0;
        boost::hash_combine( seed, e.get<0>() );
        boost::hash_combine( seed, e.get<1>() );
        boost::hash_combine( seed, e.get<2>() );
        boost::hash_combine( seed, e.get<3>() );

        return seed;
    }
};

struct iequal_toT
    : std::binary_function<TransitTuple, TransitTuple, bool>
{
    bool operator()(TransitTuple const& x, TransitTuple const& y) const
    {
        return (x.get<0>()==y.get<0>() &&
                x.get<1>()==y.get<1>() &&
                x.get<2>()==y.get<2>() &&
                x.get<3>()==y.get<3>());
    }
};

typedef boost::unordered_map< TransitTuple, int, ihashT, iequal_toT > TransitMap;
typedef TransitMap::iterator TMapIter;


typedef boost::tuples::tuple<int> InitTuple;

struct ihashInit
    : std::unary_function<InitTuple, std::size_t>
{
    std::size_t operator()(InitTuple const& e) const
    {
        std::size_t seed = 0;
        boost::hash_combine( seed, e.get<0>() );

        return seed;
    }
};

struct iequal_to_init
    : std::binary_function<InitTuple, InitTuple, bool>
{
    bool operator()(InitTuple const& x, InitTuple const& y) const
    {
        return (x.get<0>()==y.get<0>());
    }
};

typedef boost::unordered_map< InitTuple, bool, ihashInit, iequal_to_init > InitMap;
typedef InitMap::iterator IMapIter;

template<typename T>
void SaveMap(const std::string& name, T mp)
{
    std::ofstream fl(name.c_str(),std::ios_base::binary);
    boost::archive::text_oarchive oa(fl);
    boost::serialization::save(oa,mp,0);
}

template<typename T>
void LoadMap(std::string& name, T mp)
{
    std::ifstream fl(name.c_str(),std::ios_base::binary);
    boost::archive::text_iarchive oa(fl);
    boost::serialization::load(oa,mp,0);
}

/* End of maps */

/* For encoding only */

using boost::multi_index_container;
using namespace boost::multi_index;



struct TransitEntry
{
    long int frmSt;
    long int toSt;
    long long int frmAndToSt;
    StateActionTuple frmS;
    StateActionTuple toS;
    State dummyState;
    bool unsafe;
    int count;
    static long int maxDiscreteNbr;
    TransitEntry(TransitTuple tplT,int cnt)
    {
        // Transit tuples are of form <s,a,s',unsafe>
        frmSt = tplT.get<0>();
        toSt = tplT.get<2>();


        frmAndToSt = toSt + frmSt * dummyState.states;

//        frmS = s;
//        toS = sN;
        count = cnt;
        unsafe = tplT.get<3>();


    }



    friend std::ostream& operator<<(std::ostream& os,const TransitEntry& T)
    {
        // TO DO:
    }

};



/**
 Define a multi_index_container of TransitEntry, this will have indices based on s-s' nbr, s nbr and also count
 *   - a unique index sorted by fromAndToState, type long long int,
 *   - a non-unique index sorted by fromState, type long int,
 *   - a non-unique index sorted by toState, type long int
 **/



typedef multi_index_container<
  TransitEntry,
  indexed_by<
    ordered_non_unique<
      composite_key<
        TransitEntry,
        member<TransitEntry,long int,&TransitEntry::frmSt>,
        member<TransitEntry,long int,&TransitEntry::toSt>
      >,
      composite_key_compare<
        std::less<long int>,   // from
        std::less<long int> // to
      >
    >,
    ordered_unique<
      member<TransitEntry,long long int,&TransitEntry::frmAndToSt>
    >
  >
> transit_set;


#endif // MYGRAPH_H
