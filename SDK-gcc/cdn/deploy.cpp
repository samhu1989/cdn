#include "deploy.h"
#include <stack>
#include <cassert>
#include <cmath>
#include <limits>
#include <cstring>
#ifdef _DEBUG
#define PRINT   printf
#else
#define PRINT(...)
#endif
std::time_t Timer::_time_start;
const double CostLess::_relax_eps = 0.01;
CostLess::CostLess(char * topo[MAX_EDGE_NUM], int line_num):_iter_cnt(0),_ppl(nullptr)
{
    init(topo,line_num);
    toPPL();
}

CostLess::~CostLess()
{

}

void CostLess::init(char* topo[MAX_EDGE_NUM],int line_num)
{
    PRINT("CostLess::init\n");
    std::stringstream head(topo[0]);
    uint32_t c_n;
    PRINT("CostLess::init: get node_n edge_n c_n\n");
    head >> _node_n >> _edge_n >> c_n;
    PRINT("CostLess::init: node_n=%u edge_n=%u c_n=%u\n",_node_n,_edge_n,c_n);
    //reserve for nodes
    _x.reserve(_node_n);
    //create nodes
    for(uint32_t n_i = 0 ; n_i < _node_n ; ++n_i )
    {
        _x.emplace_back(n_i,-1,0);
    }
    PRINT("CostLess::init: get server price\n");
    std::stringstream price(topo[2]);
    uint32_t sp;
    price >> sp;
    _sp = sp;
    PRINT("CostLess::init: _sp=%u\n",uint32_t(_sp));
    PRINT("CostLess::init: get link information\n");
    for( uint32_t link_i = 0; link_i < _edge_n ; ++link_i )
    {
        std::stringstream link_txt( topo[4+link_i] );
        uint32_t s,e,max_band,cost;
        link_txt >> s >> e >> max_band >> cost;
        _x[s]._out_edge.emplace_back(s,e,float(max_band),float(cost));
        _x[e]._out_edge.emplace_back(e,s,float(max_band),float(cost));
        _x[s]._out_edge.back()._dual_edge = & _x[e]._out_edge.back();
        _x[e]._out_edge.back()._dual_edge = & _x[s]._out_edge.back();
    }
    PRINT("CostLess::init: get client information\n");
    _c_max = 0;
    _c_sum = 0;
    for( uint32_t c_i = 0; c_i < c_n; ++c_i)
    {
        std::stringstream c_txt( topo[4+_edge_n+1+c_i] );
        uint32_t c_id,net_id,c_traffic;
        c_txt >> c_id >> net_id >> c_traffic;
        NodeX& node = _x[net_id];
        node._ci = c_id;
        node._cx = c_traffic;
        node._ix = c_traffic;
        _c_sum += c_traffic;
        if( c_traffic > _c_max ) _c_max = c_traffic ;
    }
    PRINT("CostLess::init: End\n");
}

void CostLess::toPPL()
{
    _ppl = new SIMPLX::PPL;
    SIMPLX::PPL& ppl = *_ppl;
    uint32_t n_variables = _edge_n*2 + _node_n*3;
    PRINT("%d variables\n",n_variables);
    uint32_t n_restrictions = _edge_n*2 + _node_n*2;
    PRINT("%d restrictions\n",n_restrictions);
    ppl.restrictions_type = (SIMPLX::Restriction*) malloc(n_restrictions * sizeof(SIMPLX::Restriction));
    ppl.slack_variables = 0;
    ppl.identity_cols_indexes = (int*) malloc(n_restrictions * sizeof(SIMPLX::uint));
    for(int i = 0; i < n_restrictions; i++) {
        ppl.identity_cols_indexes[i] = -1;
    }
    ppl.A.cols = n_variables;
    ppl.A.lines = n_restrictions;
    ppl.c.size = n_variables;
    ppl.b.size = n_restrictions;

    ppl.A.data = (double*) malloc(n_variables * n_restrictions * sizeof(double));
    assert(ppl.A.data!=nullptr);
    ppl.c.data = (double*) malloc(n_variables * sizeof(double));
    ppl.b.data = (double*) malloc(n_restrictions * sizeof(double));

    SIMPLX::uint c_idx = 0;
    SIMPLX::uint b_idx = 0;
    std::vector<uint32_t> b_idx_vec; // for y = w1*0+w2*1+w3*c_sum
    b_idx_vec.reserve(_x.size());
    std::vector<NodeX>::const_iterator niter;
    for(niter=_x.cbegin();niter!=_x.cend();++niter)
    {
        const NodeX& node = *niter;
        //node cost
        SIMPLX::set_vector_value(&(ppl.c),c_idx,0);
        SIMPLX::set_matrix_value(&(ppl.A),b_idx,c_idx,1.0); //for w1+w2+w3 = 1
        SIMPLX::set_matrix_value(&(ppl.A),b_idx+1,c_idx,0.0);//for y = w1*0+w2*1+w3*c_sum
        ++c_idx;
        SIMPLX::set_vector_value(&(ppl.c),c_idx,_sp*(1.0+_relax_eps));
        SIMPLX::set_matrix_value(&(ppl.A),b_idx,c_idx,1.0); //for w1+w2+w3 = 1
        SIMPLX::set_matrix_value(&(ppl.A),b_idx+1,c_idx,1.0);//for y = w1*0+w2*1+w3*c_sum
        ++c_idx;
        SIMPLX::set_vector_value(&(ppl.c),c_idx,_sp*(1.0-_relax_eps));
        SIMPLX::set_matrix_value(&(ppl.A),b_idx,c_idx,1.0); //for w1+w2+w3 = 1
        SIMPLX::set_matrix_value(&(ppl.A),b_idx+1,c_idx,double(_c_sum)); //for y = w1*0+w2*1+w3*c_sum
        ++c_idx;
        //node constrain
        SIMPLX::set_vector_value(&(ppl.b),b_idx,1.0);   //for w1+w2+w3 = 1
        ppl.restrictions_type[b_idx] = SIMPLX::equal;
        ++b_idx;
        b_idx_vec.push_back(b_idx);
        SIMPLX::set_vector_value(&(ppl.b),b_idx,double(node._cx)); // yi + sum(xji) - sum(xij) = cx
        ppl.restrictions_type[b_idx] = SIMPLX::equal;
        for(std::vector<EdgeX>::const_iterator eiter=node._out_edge.cbegin();eiter!=node._out_edge.cend();++eiter)
        {
            const EdgeX& edge = *eiter;
            SIMPLX::set_vector_value(&(ppl.c),c_idx,double(edge._a));
            SIMPLX::set_matrix_value(&(ppl.A),b_idx,c_idx,-1.0); // yi + sum(xji) - sum(xij) = cx
            ++c_idx;
        }
        ++b_idx;
    }

    c_idx = 0;
    for(niter=_x.cbegin();niter!=_x.cend();++niter)
    {
        const NodeX& node = *niter;
        //node cost
        c_idx += 3;
        //node constrain
        for(std::vector<EdgeX>::const_iterator eiter=node._out_edge.cbegin();eiter!=node._out_edge.cend();++eiter)
        {
            const EdgeX& edge = *eiter;
            ppl.restrictions_type[b_idx] = SIMPLX::less_than;
            SIMPLX::set_matrix_value(&(ppl.A),b_idx,c_idx,1.0); // xij < max_ij
            SIMPLX::set_vector_value(&(ppl.b),b_idx,double(edge._x_max+0.001));
            SIMPLX::set_matrix_value(&(ppl.A),b_idx_vec[edge._j],c_idx,1.0); // yi + sum(xji) - sum(xij) = cx
            ++c_idx;
            ++b_idx;
        }
    }
    SIMPLX::print_PPL(_ppl);
}

uint32_t CostLess::less()
{
    // Initializing structs with zero values
    SIMPLX::Simplex_table simplex = {};
    SIMPLX::print_PPL(_ppl);
    SIMPLX::expand_PPL(_ppl);
    SIMPLX::print_PPL(_ppl);
    if(SIMPLX::get_first_phase_table(&simplex,_ppl)) {
        run_simplex(&simplex,0,0);
        PRINT(" ^\n |\nFirst phase solution\n");
        if(get_second_phase_table(&simplex,_ppl) == SIMPLX::unfeasible)
        {
            PRINT("NA");
        }
    }
    run_simplex(&simplex,0,0);
    if(SIMPLX::lpp_type_from_solved_table(&simplex) == SIMPLX::mutiple_solution)
    {
        PRINT("Multiple Solution\n");
        SIMPLX::Remembered_bases remembered_bases = {};
        SIMPLX::alloc_remembered_bases(&remembered_bases,simplex.n_variables_in_base);
        SIMPLX::print_other_solutions_from_base_solution(&simplex, &remembered_bases);
    }
    else {
        PRINT("Single Solution\n");
        SIMPLX::print_simplex_table(&simplex);
    }
    ++_iter_cnt;
}

bool CostLess::isEnd()
{
    return _iter_cnt >= 1;
}

const char* CostLess::getRes()
{
    PRINT("CostLess::getRes()\n");
    std::stringstream stream("");
    uint32_t cnt = 0;
    std::vector<NodeX>::const_iterator niter;
    for(niter=_x.cbegin();niter!=_x.cend();++niter)
    {
        if(niter->_ix==0)continue;
        std::stack<uint32_t> node_to_visit;
        std::vector<uint32_t> current_path;
        current_path.reserve(_x.size());
        std::vector<uint32_t> min_traffic; //minimum traffic when reach current node by current path
        min_traffic.reserve(_x.size());
        std::vector<bool> visited(_x.size(),false);
        node_to_visit.push(niter->_n);
        while(!node_to_visit.empty())
        {
            uint32_t node_visit = node_to_visit.top();
            PRINT("CostLess::getRes():visiting n(%u)\n",node_visit);
            visited[node_visit] = true;
            node_to_visit.pop();
            const NodeX &node = _x[node_visit];
            if(current_path.empty())
            {
                current_path.push_back(node._n);
                min_traffic.push_back(node._ix);
            }else{
                const NodeX& node_from = _x[current_path.back()];
                for(std::vector<EdgeX>::const_iterator eiter=node_from._out_edge.cbegin();eiter!=node_from._out_edge.cend();++eiter)
                {
                    const EdgeX & edge = *eiter;
                    if( edge._j == node._n  )
                    {
                        if( edge._ix < min_traffic.back() )
                        {
                            min_traffic.push_back( edge._ix );
                        }else{
                            min_traffic.push_back(min_traffic.back());
                        }
                    }
                }
                current_path.push_back(node._n);
            }
            //arrived at a consumer
            if( -1 != node._ci && min_traffic.back() > 0 )
            {
                for(std::vector<uint32_t>::const_iterator piter=current_path.cbegin();piter!=current_path.cend();++piter)
                {
                    stream << *piter <<" ";
                }
                stream << node._ci <<" ";
                if( node._cx < min_traffic.back() )
                {
                    stream << node._cx <<"\n";
                }else{
                    stream << min_traffic.back() <<"\n";
                }
                if( current_path.size() > 1 )
                {
                    current_path.pop_back();
                    min_traffic.pop_back();
                }
                ++cnt;
            }
            if( min_traffic.back() > 0 )
            {
                for(std::vector<EdgeX>::const_iterator eiter=node._out_edge.cbegin();eiter!=node._out_edge.cend();++eiter)
                {
                    const EdgeX& edge = *eiter;
                    if( 0 != edge._ix && !visited[edge._j] )
                    {
                        node_to_visit.push(edge._j);
                    }
                }
            }
        }
    }
    _res_txt<<cnt<<"\n\n";
    _res_txt<<stream.str();
    _res_str = _res_txt.str();
    PRINT("CostLess::getRes():\n%s\n",_res_str.c_str());
    return _res_str.c_str();
}

//You need to complete the function 
void deploy_server(char * topo[MAX_EDGE_NUM], int line_num,char * filename)
{
	Timer::tic();
    CostLess cost(topo,line_num);
    while( !cost.isEnd() && !Timer::timeout() )
    {
        uint32_t c = cost.less();
        PRINT("Cost:%u\n",c);
    }
    const char* res = cost.getRes();
    PRINT("deploy_server:\n%s\n",res);
    write_result(res,filename);
}

namespace SIMPLX {
#define TRUE 1
#define FALSE 0
#define abs(n) (n < 0 ? -n : n)
//TODO: create macros to get PPL variables (and restrictions) number
//TODO: give priority to artificial variables to exit the base

int get_PPL_from_file(FILE* input, PPL* ppl) {
    uint n_variables;
    uint n_restrictions;

    PRINT("# of variables: ");
    fscanf(input, "%d", &n_variables);

    PRINT("\n# of restrictions: ");
    fscanf(input, "%d", &n_restrictions);

    ppl->restrictions_type = (Restriction*)malloc(n_restrictions * sizeof(Restriction));
    for(uint i = 0; i < n_restrictions; i++) {
        PRINT("\nRestriction type ( < | > | = ): ");
        char tmp_char = 0;
        do {
            fscanf(input, "%c", &tmp_char);
        } while(tmp_char != '<' && tmp_char != '>' && tmp_char != '=');

        ppl->restrictions_type[i] = (Restriction) tmp_char;
    }
    ppl->slack_variables = 0;
    ppl->identity_cols_indexes = (int*)malloc(n_restrictions * sizeof(uint));

    for(uint i = 0; i < n_restrictions; i++) {
        ppl->identity_cols_indexes[i] = -1;
    }

    ppl->A.cols = n_variables;
    ppl->A.lines = n_restrictions;
    ppl->c.size = n_variables;
    ppl->b.size = n_restrictions;

    ppl->A.data = (double*) malloc(n_variables * n_restrictions * sizeof(double));
    ppl->c.data = (double*) malloc(n_variables * sizeof(double));
    ppl->b.data = (double*) malloc(n_restrictions * sizeof(double));

    PRINT("Enter the function coefficients\n");
    for(uint i = 0; i < n_variables; i++) {
        double tmp_double;
        fscanf(input, "%lf", &tmp_double);
        set_vector_value(&(ppl->c), i, tmp_double);
    }

    PRINT("Enter the \"A\" matrix values (per line)\n");
    for(uint i = 0; i < n_restrictions; i++) {
        for(uint j = 0; j < n_variables; j++) {
            double tmp_double;
            fscanf(input, "%lf", &tmp_double);
            set_matrix_value(&(ppl->A), i, j, tmp_double);
        }
    }

    PRINT("Enter the \"b\" vector values\n");
    for(uint i = 0; i < n_restrictions; i++) {
        double tmp_double;
        fscanf(input, "%lf", &tmp_double);
        set_vector_value(&(ppl->b), i, tmp_double);
    }

    return 1;
}

void print_PPL(PPL* ppl) {
    Vector* c = &(ppl->c);
    Vector* b = &(ppl->b);
    Matrix* A = &(ppl->A);

    PRINT("Min\n    ");
    print_vector_as_coefficients(c);

    PRINT("\nsubject to\n");
    for(uint i = 0; i < A->lines; i++) {
        Vector tmp_vector;
        get_vector_from_matrix_line(A, &tmp_vector, i);
        PRINT("    ");
        print_vector_as_coefficients(&tmp_vector);

        PRINT(" %c %.2lf\n", ppl->restrictions_type[i], get_vector_value(b, i));
    }
}

void print_vector_as_coefficients(Vector* vector) {
    if(vector->size == 0) return;

    PRINT("%.2lf x%d", get_vector_value(vector, 0), 0);
    for(uint i = 1; i < vector->size; i++) {
        double value = get_vector_value(vector, i);
        if(value < 0)
            PRINT(" - ");
        else
            PRINT(" + ");

        PRINT("%.2lf x%d", abs(value), i);
    }
}

void print_simplex_table(Simplex_table* simplex_table) {
    PRINT("    | z | ");
    for(uint i = 0; i < simplex_table->table.cols - 1; i++)
        PRINT("  x%d       ", i);
    PRINT("| RHS\n");

    for(uint i = 0; i < simplex_table->table.cols; i++)
        PRINT("============");
    PRINT("\n");

    PRINT(" z  | 1 | ");
    {
        uint i;
        for(i = 0; i < simplex_table->table.cols - 1; i++){
            PRINT(" %-9.2lf ", get_matrix_value(&(simplex_table->table), 0, i));
        }
        PRINT("| %-9.2lf\n", get_matrix_value(&(simplex_table->table), 0, i));
    }

    for(uint i = 0; i < simplex_table->table.cols; i++)
        PRINT("============");
    PRINT("\n");

    for(uint i = 0; i < simplex_table->n_variables_in_base; i++) {
        PRINT(" x%d | 0 | ", simplex_table->variables_in_base[i]);
        uint j;
        for(j = 0; j < simplex_table->table.cols - 1; j++) {
            PRINT(" %-9.2lf ", get_matrix_value(&(simplex_table->table), i + 1, j));
        }
        PRINT("| %-9.2lf\n", get_matrix_value(&(simplex_table->table), i + 1, j));
    }
}

inline void set_matrix_value(Matrix* matrix, uint i, uint j, double value) {
    matrix->data[i * matrix->cols + j] = value;
}

inline void set_vector_value(Vector* vector, uint i, double value) {
    vector->data[i] = value;
}

inline double get_matrix_value(Matrix* matrix, uint i, uint j) {
    return matrix->data[i * matrix->cols + j];
}

inline double get_vector_value(Vector* vector, uint i) {
    return vector->data[i];
}

inline void get_vector_from_matrix_line(Matrix* matrix, Vector* vec, uint line) {
    vec->size = matrix->cols;
    vec->data = matrix->data + (line * matrix->cols);
}

void get_vector_from_matrix_col_with_offset(Matrix* matrix, Vector* vector, uint offset, uint col) {
    for(uint i = 0; i < vector->size; i++) {
        double value = get_matrix_value(matrix, i + offset, col);
        set_vector_value(vector, i, value);
    }
}

void copy_matrix_with_line_offset(Matrix* dest, Matrix* source, uint dest_offset, uint source_offset) {
    for(uint source_line = source_offset, dest_line = dest_offset;
        dest_line < dest->lines && source_line < source->lines;
        source_line++, dest_line++) {
        for(uint j = 0; j < dest->cols && j < source->cols; j++) {
            double value = get_matrix_value(source, source_line, j);
            set_matrix_value(dest, dest_line, j, value);
        }
    }
}

void copy_vector(Vector* dest, Vector* source) {
    for(uint i = 0; i < source->size; i++) {
        dest->data[i] = source->data[i];
    }
}

double inner_product(Vector* a, Vector* b) {
    double result = 0.0;
    for(uint i = 0; i < a->size; i++) {
        //TODO(maybe): Use pointer arithmetic here
        result += a->data[i] * b->data[i];
    }

    return result;
}

void sum_vector_with_multiplier(Vector* dest, Vector* source, double multiplier) {
    for(uint i = 0; i < dest->size; i++) {
        double value = get_vector_value(dest, i) + (get_vector_value(source, i) * multiplier);
        set_vector_value(dest, i, value);
    }
}

void divide_vector_by_scalar(Vector* vector, double scalar) {
    for(uint i = 0; i < vector->size; i++) {
        double value = get_vector_value(vector, i) / scalar;
        set_vector_value(vector, i, value);
    }
}

void expand_PPL(PPL* ppl) {
    // using namespace PPL
    Matrix* A = &(ppl->A);
    Vector* c = &(ppl->c);

    // For each restriction inequality restriction, we will have a slack variable
    for(uint i = 0; i < ppl->b.size; i++) {
        if(ppl->restrictions_type[i] != equal)
            ppl->slack_variables++;
    }

    int new_n_varibles = A->cols + ppl->slack_variables;

    Matrix new_matrix;
    new_matrix.cols = new_n_varibles;
    new_matrix.lines = A->lines;
    new_matrix.data = (double*) malloc(A->lines * new_n_varibles * sizeof(double));

    // Copying the old matrix to the new one
    for(uint i = 0; i < A->lines; i++) {
        for(uint j = 0; j < A->cols; j++) {
            set_matrix_value(&new_matrix, i, j, get_matrix_value(A, i, j));
        }
    }

    uint n_slack_variables_added = 0;
    for(uint i = 0; i < A->lines; i++) {
        for(uint j = 0; j < ppl->slack_variables; j++) {
            uint col = A->cols + j;
            double new_value = 0.0;
            if(ppl->restrictions_type[i] != equal && n_slack_variables_added == j) {
                switch(ppl->restrictions_type[i]) {
                    case less_than:
                        new_value = 1.0;
                        ppl->identity_cols_indexes[i] = col;
                        break;
                    case greater_than:
                        new_value = -1.0;
                        break;
                    default:
                        break;
                }
                n_slack_variables_added++;
                ppl->restrictions_type[i] = equal;
            }

            set_matrix_value(&new_matrix, i, col, new_value);
        }
    }

    free(A->data);
    ppl->A = new_matrix;

    c->data = (double*)realloc(c->data, new_n_varibles * sizeof(double));
    for(uint i = c->size; i < new_n_varibles; i++) {
        set_vector_value(c, i, 0.0);
    }

    c->size = new_n_varibles;
}

void get_c_b(Simplex_table* simplex_table, Vector* c_b) {
    for(uint i = 0; i < simplex_table->n_variables_in_base; i++) {
        c_b->data[i] = get_vector_value(&(simplex_table->costs), simplex_table->variables_in_base[i]);
    }
}

void fill_simplex_table_z_line(Simplex_table* simplex_table, Vector* c_b) {
    Vector tmp_vector;
    tmp_vector.size = c_b->size;
    tmp_vector.data = (double*) malloc(tmp_vector.size * sizeof(double));

    uint line_offset = 1;
    for(uint i = 0; i < simplex_table->costs.size; i++) {
        // B^-1 * a_j is already in the table columns, therefore B^-1 doesn't need to be calculated
        get_vector_from_matrix_col_with_offset(&(simplex_table->table), &tmp_vector, line_offset, i);
        double minus_reduced_cost = inner_product(c_b, &tmp_vector) -
                                    get_vector_value(&(simplex_table->costs), i);
        set_matrix_value(&(simplex_table->table), 0, i, minus_reduced_cost);
    }

    uint last_col_index = simplex_table->table.cols - 1;
    get_vector_from_matrix_col_with_offset(&(simplex_table->table), &tmp_vector, line_offset, last_col_index);
    double z_value = inner_product(c_b, &tmp_vector);
    set_matrix_value(&(simplex_table->table), 0, last_col_index, z_value);

    free(tmp_vector.data);
}

// This function returns a zero value if the 2-phase method is needed, otherwise it returns a non-zero value.
// In the former case the 'simplex_table' is prepared to run the first phase, in the latter it is prepared to
// run the second phase directly.
int get_first_phase_table(Simplex_table* simplex_table, PPL* ppl) {
    uint n_artificial_variables = 0;

    simplex_table->n_variables_in_base = ppl->b.size;
    simplex_table->variables_in_base = (uint*) malloc(simplex_table->n_variables_in_base * sizeof(uint));

    // The identity_cols_indexes were filled by calling expand_PPL
    for(uint i = 0; i < simplex_table->n_variables_in_base; i++) {
        if(ppl->identity_cols_indexes[i] == -1) {
            // The artificial variables will occupy the last columns of the simplex_table and will be put in the base
            simplex_table->variables_in_base[i] = ppl->c.size + n_artificial_variables;
            n_artificial_variables++;
        }
        else {
            // A slack variable already has an indentity column, so this variable is put in the base
            simplex_table->variables_in_base[i] = ppl->identity_cols_indexes[i];
        }
    }

    //TODO: Create an allocate_vector function
    simplex_table->costs.size = ppl->A.cols + n_artificial_variables;
    simplex_table->costs.data = (double*) calloc(simplex_table->costs.size, sizeof(double));

    simplex_table->table.cols = simplex_table->costs.size + 1;
    simplex_table->table.lines = ppl->A.lines + 1;

    size_t table_size = simplex_table->table.cols * simplex_table->table.lines;
    simplex_table->table.data = (double*) calloc(table_size, sizeof(double));

    copy_matrix_with_line_offset(&(simplex_table->table), &(ppl->A), 1, 0);

    // Sets the artificial variables columns to the correspondent indentity column
    // Since the table is filled with zeroes by calloc only the ones are filled
    for(uint i = 0; i < simplex_table->n_variables_in_base; i++) {
        uint variable_index = simplex_table->variables_in_base[i];
        if(variable_index >= ppl->c.size) // if this varible is an artificial variable
            set_matrix_value(&(simplex_table->table), i + 1, variable_index, 1.0);
    }

    uint last_table_col_index = simplex_table->table.cols - 1;
    //TODO(Maybe): Refactor to copy_vector_to_matrix_col
    for(uint i = 0; i < ppl->b.size; i++) {
        set_matrix_value(&(simplex_table->table), i + 1, last_table_col_index, get_vector_value(&(ppl->b), i));
    }

    Vector c_b;
    c_b.size = simplex_table->n_variables_in_base;
    c_b.data = (double*) malloc(c_b.size * sizeof(double));

    // Fill the costs vector in the simplex_table
    if(n_artificial_variables) {
        // The costs vector is guaranteed to be all zeroes by calloc
        for(uint i = ppl->c.size; i < ppl->c.size + n_artificial_variables; i++) {
            set_vector_value(&(simplex_table->costs), i, 1.0);
        }
    }
    else { //If there is no artificial variable, the costs vector is the problem c vector
        copy_vector(&(simplex_table->costs), &(ppl->c));
    }

    get_c_b(simplex_table, &c_b);

    fill_simplex_table_z_line(simplex_table, &c_b);

    // TODO(Maybe): try to reuse c_b so it doesn't need to be freed and reallocated
    free(c_b.data);

    //README: By returning n_artificial_variables we achieve the function requirement of returning zero if the
    //        2-phase method is not need.
    return n_artificial_variables;
}

LPP_type get_second_phase_table(Simplex_table* simplex_table, PPL* ppl) {
    uint n_original_variables = ppl->c.size;

    for(uint i = 0; i < simplex_table->n_variables_in_base; i++) {
        // We couldn't remove an artificial variable from the base. The problem is unfeasible
        if(simplex_table->variables_in_base[i] >= n_original_variables)
            return unfeasible;
    }

    Vector new_costs;
    Vector c_b;
    Matrix new_table;

    new_costs.size = n_original_variables;
    c_b.size = simplex_table->n_variables_in_base;
    new_table.lines = simplex_table->table.lines;
    new_table.cols = n_original_variables + 1;

    new_costs.data = (double*) malloc(new_costs.size * sizeof(double));
    c_b.data = (double*) malloc(c_b.size * sizeof(double));
    new_table.data = (double*) malloc(new_table.cols * new_table.lines * sizeof(double));

    copy_vector(&new_costs, &(ppl->c));
    copy_matrix_with_line_offset(&new_table, &(simplex_table->table), 1, 1);

    // Copying b_bar
    uint old_last_col_index = simplex_table->table.cols - 1;
    uint new_last_col_index = new_table.cols - 1;
    for(uint i = 1; i < new_table.lines; i++) {
        double value = get_matrix_value(&(simplex_table->table), i, old_last_col_index);
        set_matrix_value(&new_table, i, new_last_col_index, value);
    }

    // the old table is no longer needed.
    free(simplex_table->table.data);
    free(simplex_table->costs.data);
    //The simplex table is updated so we can continue filling it.
    simplex_table->table = new_table;
    simplex_table->costs = new_costs;


    get_c_b(simplex_table, &c_b);

    fill_simplex_table_z_line(simplex_table, &c_b);

    return feasible;
}

int get_exiting_variable(Simplex_table* simplex_table, int entering_varible_index) {
    int exiting_varible_index_in_base = -1;
    double exting_variable_fraction = 0.0;

    for(uint i = 0; i < simplex_table->n_variables_in_base; i++) {
        double col_value = get_matrix_value(&(simplex_table->table), i + 1, entering_varible_index);
        if(col_value <= 0)
            continue;

        double fraction = get_matrix_value(&(simplex_table->table), i + 1, simplex_table->costs.size) / col_value;
        // TODO: use bland method, if needed
        if(exiting_varible_index_in_base == -1 || fraction < exting_variable_fraction) {
            exting_variable_fraction = fraction;
            exiting_varible_index_in_base = i;
        }
    }

    return exiting_varible_index_in_base;
}

LPP_type run_simplex(Simplex_table* simplex_table, int should_print_all_tables, int should_use_bland) {
    LPP_type result = feasible;

    while(1){
        int entering_varible_index = -1;
        double entering_varible_minus_reduce_cost = 0.0;
        int exiting_varible_index_in_base;

        for(uint i = 0; i < simplex_table->costs.size; i++) {
            double minus_reduced_cost = get_matrix_value(&(simplex_table->table), 0, i);
            if(minus_reduced_cost > 0.0) {
                // TODO: use bland method, if needed
                if(minus_reduced_cost > entering_varible_minus_reduce_cost) {
                    entering_varible_minus_reduce_cost = minus_reduced_cost;
                    entering_varible_index = i;
                }
            }
        }

        if(entering_varible_index == -1) // No variable to enter the base
            break;

        exiting_varible_index_in_base = get_exiting_variable(simplex_table, entering_varible_index);

        // README: if(exiting_variable_index != -1 && exitiing_variable_fraction == 0) // The base is degenerated

        if(exiting_varible_index_in_base == -1){ // README: Unbounded solution? Not sure
            result = unbounded;
            break;
        }

        if(should_print_all_tables) {
            PRINT("Entering: x%d -- with value %0.24lf\nExiting: x%d\n", entering_varible_index,
                    entering_varible_minus_reduce_cost,
                    simplex_table->variables_in_base[exiting_varible_index_in_base]);
        }

        put_in_base(simplex_table, entering_varible_index, exiting_varible_index_in_base); // a.k.a. Pivot
        if(should_print_all_tables) {
            print_simplex_table(simplex_table);
        }
    }

    return result;
}

void put_in_base(Simplex_table* simplex_table, uint entering, uint exiting) {
    Vector pivot_line;
    Vector zeroing_line;
    get_vector_from_matrix_line(&(simplex_table->table), &pivot_line, exiting + 1);

    double pivot = get_vector_value(&pivot_line, entering);

    for(uint i = 0; i < simplex_table->table.lines; i++) {
        if(i == exiting + 1) continue;

        get_vector_from_matrix_line(&(simplex_table->table), &zeroing_line, i);
        double m = -(get_vector_value(&zeroing_line, entering) /
                     pivot);

        sum_vector_with_multiplier(&zeroing_line, &pivot_line, m);
    }

    divide_vector_by_scalar(&pivot_line, pivot);
    simplex_table->variables_in_base[exiting] = entering;
}

int is_variable_in_base(Simplex_table* simplex_table, uint variable) {
    for(uint i = 0; i < simplex_table->n_variables_in_base; i++) {
        if(variable == simplex_table->variables_in_base[i]) {
            return TRUE;
        }
    }
    return FALSE;
}

LPP_type lpp_type_from_solved_table(Simplex_table* simplex_table) {
    Matrix* table = &(simplex_table->table);
    for(uint i = 0; i < simplex_table->table.cols - 1; i++) {
        if(get_matrix_value(table, 0, i) == 0.0) {
            if(!is_variable_in_base(simplex_table, i)) return mutiple_solution;
        }
    }
    return single_solution;
}

void print_other_solutions_from_base_solution(Simplex_table* simplex_table, Remembered_bases* remembered_bases) {
    copy_to_tmp_base(remembered_bases, simplex_table->variables_in_base);
    remember_tmp_base(remembered_bases);

    Matrix* table = &(simplex_table->table);
    for(uint i = 0; i < simplex_table->table.cols - 1; i++) {
        if(get_matrix_value(table, 0, i) == 0.0) {
            if(!is_variable_in_base(simplex_table, i)) {
                uint entering_variable = i;
                int exiting_varible_index_in_base = get_exiting_variable(simplex_table, entering_variable);
                if(exiting_varible_index_in_base == -1) continue;

                if(is_base_remembered(remembered_bases,
                                      simplex_table->variables_in_base,
                                      exiting_varible_index_in_base,
                                      entering_variable)) continue;

                put_in_base(simplex_table, entering_variable, exiting_varible_index_in_base);
                print_simplex_table(simplex_table);
                print_other_solutions_from_base_solution(simplex_table, remembered_bases);
                break;
            }
        }
    }
}

void alloc_remembered_bases(Remembered_bases* remembered_bases, uint variables_per_base) {
    uint default_capacity = 10;
    remembered_bases->capacity_in_bases = default_capacity;
    remembered_bases->n_bases = 0;
    remembered_bases->variables_per_base = variables_per_base;

    remembered_bases->tmp_base = (uint*) malloc(variables_per_base * sizeof(uint));
    remembered_bases->values = (uint*) malloc(variables_per_base * default_capacity * sizeof(uint));

}

void realloc_remembered_bases(Remembered_bases* remembered_bases) {
    remembered_bases->capacity_in_bases *= 2;

    remembered_bases->values = (uint*) realloc(remembered_bases->values, remembered_bases->capacity_in_bases *
                                                                         remembered_bases->variables_per_base *
                                                                         sizeof(uint));
}

void sort_tmp_base(Remembered_bases* remembered_bases) {
    uint* tmp_base = remembered_bases->tmp_base;
    for(uint i = 0; i < remembered_bases->variables_per_base; i++) {
        for(uint j = 0; j < remembered_bases->variables_per_base - i - 1; j++) {
            if(tmp_base[j] > tmp_base[j + 1]){
                uint tmp = tmp_base[j];
                tmp_base[j] = tmp_base[j + 1];
                tmp_base[j + 1] = tmp;
            }
        }
    }
}

void remember_tmp_base(Remembered_bases* remembered_bases) {
    sort_tmp_base(remembered_bases);

    if(remembered_bases->n_bases == remembered_bases->capacity_in_bases)
        realloc_remembered_bases(remembered_bases);

    uint current_index = remembered_bases->n_bases * remembered_bases->variables_per_base;

    for(uint i = 0; i < remembered_bases->variables_per_base; i++) {
        remembered_bases->values[current_index++] = remembered_bases->tmp_base[i];
    }

    remembered_bases->n_bases++;
}

void copy_to_tmp_base(Remembered_bases* remembered_bases, uint* base) {
    for(uint i = 0; i < remembered_bases->variables_per_base; i++) {
        remembered_bases->tmp_base[i] = base[i];
    }
}

int is_same_base(uint* base_a, uint* base_b, uint base_size) {
    for(uint i = 0; i < base_size; i++) {
        if(base_a[i] != base_b[i]) return FALSE;
    }
    return TRUE;
}

int is_base_remembered(Remembered_bases* remembered_bases, uint* base, uint changing_varible, uint new_variable) {
    copy_to_tmp_base(remembered_bases, base);
    remembered_bases->tmp_base[changing_varible] = new_variable;

    sort_tmp_base(remembered_bases);

    uint variables_per_base = remembered_bases->variables_per_base;
    for(uint i = 0; i < remembered_bases->n_bases; i++) {
        uint* current_base = remembered_bases->values + (i * variables_per_base);
        if(is_same_base(remembered_bases->tmp_base, current_base, variables_per_base))
            return TRUE;
    }

    return FALSE;
}

int test_simplex(void){
    PRINT("testing simplex:");
    int should_print_all_tables = TRUE;
    int should_use_bland = FALSE;

    // Initializing structs with zero values
    PPL ppl = {};
    Simplex_table simplex = {};

    get_PPL_from_file(stdin, &ppl);

    PRINT("\n\n\n");
    print_PPL(&ppl);

    expand_PPL(&ppl);
    print_PPL(&ppl);

    if(get_first_phase_table(&simplex, &ppl)) {
        PRINT("\n\n\n");
        print_simplex_table(&simplex);

        run_simplex(&simplex, should_print_all_tables, should_use_bland);
        if(!should_print_all_tables) print_simplex_table(&simplex);
        PRINT(" ^\n |\nFirst phase solution\n");
        //TODO: Tell the user that the problem is unfeasible
        if(get_second_phase_table(&simplex, &ppl) == unfeasible) exit(-1);
    }
    PRINT("\n\n\n");
    print_simplex_table(&simplex);
    // TODO: Check to see if it is an unbounded problem
    run_simplex(&simplex, should_print_all_tables, should_use_bland);

    if(!should_print_all_tables) print_simplex_table(&simplex);
    PRINT(" ^\n |\n");

    if(lpp_type_from_solved_table(&simplex) == mutiple_solution) {
        PRINT("Multiple Solution\n");
        PRINT("Others solutions are:\n");
        Remembered_bases remembered_bases = {};
        alloc_remembered_bases(&remembered_bases, simplex.n_variables_in_base);
        print_other_solutions_from_base_solution(&simplex, &remembered_bases);
    }
    else {
        PRINT("Single Solution\n");
    }

    return 0;
}
}
