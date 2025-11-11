#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <exception>
#include <queue>
#include <cstring>
#include <cassert>
#include "GenericPlanner.cpp"

#define DEBUG 0
#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6
#ifndef ENVS_DIR
#define ENVS_DIR "../envs"
#endif

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

/////////////////// INITIAL DECLARATIONS ///////////////////////
template <typename T>
static std::vector<T> list2vector(const std::list<T>& list ) {
    return std::vector<T>(list.begin(), list.end());
}

template <typename T>
static std::list<T> vector2list(const std::vector<T>& vector ) {
    return std::list<T>(vector.begin(), vector.end());
}

vector<vector<string>> getAllSymbolPermutations(const vector<string>& items, int k) {
    int n = items.size();
    if (k < 0 || k > n) {
        throw std::runtime_error("Generating permutations when k=0??");
    }

    vector<vector<string>> result;
    vector<string> cur;
    cur.reserve(k);
    vector<bool> used(n, false);

    function<void()> dfs = [&]() {
        if ((int)cur.size() == k) {
            result.push_back(cur);
            return;
        }
        for (int i = 0; i < n; ++i) {
            if (used[i]) continue;
            used[i] = true;
            cur.push_back(items[i]);
            dfs();
            cur.pop_back();
            used[i] = false;
        }
    };

    dfs();
    return result;
}

using UMap_StrStr = unordered_map<string, string>;
/////////////////// INITIAL DECLARATIONS //////////////////////////

/////////////////// Condition and Grounded Conditions /////////////
/** @brief This is a generic condition
 * Consists of a string logical predicate that can be evaluated to True or False,
 * An argument list that the condition applied to and
 * a boolean true or false value for this proposition.
 */
class Condition
{
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(const string &pred, const list<string> &args, const bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (const string &ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream &operator<<(ostream &os, const Condition &cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition &rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (const string &l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition &lhs, const Condition &rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition &cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

/**
 * GroundedCondition has actual variables from the current environment
 *
 */
class GroundedCondition
{
    string predicate;
    list<string> arg_values;
    UMap_StrStr substitutions;
    bool truth = true;

public:

    GroundedCondition(const string &predicate, const list<string> &arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth; // fixed
        for (const string &l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition &gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth; // fixed
        for (const string &l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const Condition& genericCondition, list<string>& arg_vals);

    void set_substitutions(const UMap_StrStr& subs) {
        this->substitutions = subs;
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream &operator<<(ostream &os, const GroundedCondition &pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition &rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp;
        temp += this->predicate;
        temp += "(";
        for (const string &l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition &lhs, const GroundedCondition &rhs) const
    {
        return lhs.get_arg_values() == rhs.get_arg_values() &&
            lhs.get_predicate() == rhs.get_predicate();;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition &gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

static std::unordered_map<std::string, std::string> buildConditionSubst(const Condition& condition, std::list<std::string>& ground_args) {
    if (condition.get_args().size() != ground_args.size()) {
        throw std::runtime_error("Mismatch when building subst map for ConditionSubstitution");
    }

    std::unordered_map<std::string, std::string> substitutions;
    for (auto generic_it = condition.get_args().begin(), grounded_args_it = ground_args.begin();
        generic_it != condition.get_args().end();
        ++generic_it, ++grounded_args_it) {

        substitutions[*generic_it] = *grounded_args_it;
    }
    return substitutions;
}

static GroundedCondition performConditionSubstitution(const Condition& in, const std::unordered_map<std::string,std::string>& substitutionArgs) {
    std::list<string> groundedArgs;

    for (const std::string& arg : in.get_args()) {
        auto it = substitutionArgs.find(arg);
        if (it != substitutionArgs.end())
            groundedArgs.emplace_back(it->second);
        else
            groundedArgs.push_back(arg);
    }

    return GroundedCondition(in.get_predicate(), groundedArgs, in.get_truth());
}

GroundedCondition::GroundedCondition(const Condition& genericCondition, list<string>& grounded_args) {
    this->predicate = genericCondition.get_predicate();
    this->truth = genericCondition.get_truth();
    this->substitutions = buildConditionSubst(genericCondition, grounded_args);
    this->arg_values = performConditionSubstitution(genericCondition, this->substitutions).get_arg_values();
}
///////////////////////////// Condition and Grounded Conditions ////////////////////////////////////

/////////////////////////////// Actions and Grounded Actions ///////////////////////////////////////
/**
 * Consists of the action name, arguments, precondition, and effects
 * Arguments are what the action acts on
 * Preconditions are things that must be satisfied
 * Effects are things that change when this action is applied
 */
class Action
{
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(const string &name, const list<string> &args,
           const unordered_set<Condition, ConditionHasher, ConditionComparator> &preconditions,
           const unordered_set<Condition, ConditionHasher, ConditionComparator> &effects)
    {
        this->name = name;
        for (const string &l : args)
        {
            this->args.push_back(l);
        }
        for (const Condition &pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (const Condition &pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action &rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream &operator<<(ostream &os, const Action &ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (const Condition &precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (const Condition &effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->get_name();
        temp += "(";
        for (const string &l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action &lhs, const Action &rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action &ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

using USet_action = unordered_set<GroundedAction, ActionHasher, ActionComparator>;
using USet_cond = unordered_set<Condition, ConditionHasher, ConditionComparator>;
using USet_gcond = unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>;

class GroundedAction
{
    string name;
    list<string> arg_values;
    USet_gcond groundedPreconditions;
    USet_gcond groundedEffects;
    UMap_StrStr substitutions;

public:
    GroundedAction(const string &name, const list<string> &arg_values)
    {
        this->name = name;
        for (const string &ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    GroundedAction(const Action&, const list<string>&);

    void setGPreconditions(const Action& action);

    void setGEffects(const Action& action);

    void setSubstitutions(const UMap_StrStr& subs) {
        this->substitutions = subs;
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    USet_gcond getGroundedPreconditions() const {
        return this->groundedPreconditions;
    }

    USet_gcond getGroundedEffects() const {
        return this->groundedEffects;
    }

    bool operator==(const GroundedAction &rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream &operator<<(ostream &os, const GroundedAction &gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp;
        temp += this->name;
        temp += "(";
        for (const string &l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedActionComparator
{
    bool operator()(const GroundedAction &lhs, const GroundedAction &rhs) const
    {
        return lhs.get_arg_values() == rhs.get_arg_values() &&
            lhs.get_name() == rhs.get_name();
    }
};

struct GroundedActionHasher // TODO This is only hasing the name. Shouldnt the args also be hashed?
{
    size_t operator()(const GroundedAction &ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

using USet_gaction = unordered_set<GroundedAction, GroundedActionHasher, GroundedActionComparator>;

static std::unordered_map<std::string,std::string> buildActionSubst(const Action& action, const std::list<std::string>& grounded_vals) {
    if (action.get_args().size() != grounded_vals.size()) {
        throw std::runtime_error("Mismatch in number of arguments when building substitution map for action " + action.get_name());
    }

    std::unordered_map<std::string,std::string> subst;
    const std::list<string>& gvals = action.get_args();
    auto gvals_it = grounded_vals.begin();
    auto generic_it = gvals.begin();

    for (;generic_it != gvals.end();
        ++gvals_it, ++generic_it) {
        subst[*generic_it] = *gvals_it;
    }
    return subst;
}

static GroundedAction performActionSubstitution(const Action& in, std::unordered_map<std::string,std::string>& substitutionArgs) {
    if (in.get_args().size() != substitutionArgs.size()) {
        throw std::runtime_error("Mismatch in number of arguments to ActionSubstitution");
    }

    std::vector<string> substituted;
    for (auto & it : in.get_args()) {
        if ( substitutionArgs.find(it) != substitutionArgs.end())
            substituted.push_back(it);
        else
            substituted.push_back(substitutionArgs[it]);
    }

    return GroundedAction(in.get_name(), vector2list(substituted));
}

void GroundedAction::setGPreconditions(const Action& action) {
    for (const Condition& genericCond: action.get_preconditions()) {
        GroundedCondition gPreCond = performConditionSubstitution(genericCond, this->substitutions);
        this->groundedPreconditions.insert(gPreCond);
    }
}

void GroundedAction::setGEffects(const Action& action) {
    for (const Condition& genericCond: action.get_effects()) {
        GroundedCondition gEffect = performConditionSubstitution(genericCond, this->substitutions);
        this->groundedEffects.insert(gEffect);
    }
}

GroundedAction::GroundedAction(const Action& genericAction, const list<string>& arg_vals) {
    name = genericAction.get_name();
    arg_values = arg_vals;
    substitutions = buildActionSubst(genericAction, arg_vals);
    setGPreconditions(genericAction);
    setGEffects(genericAction);
}

/////////////////////////////// Actions and Grounded Actions ///////////////////////////////////////

class Env
{
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(const GroundedCondition &gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(const GroundedCondition &gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(const GroundedCondition &gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(const GroundedCondition &gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(const string &symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(const list<string> &symbols)
    {
        for (const string &l : symbols)
            this->symbols.insert(l);
    }
    void add_action(const Action &action)
    {
        this->actions.insert(action);
    }

    Action get_action(const string &name) const
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }

    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    friend ostream &operator<<(ostream &os, const Env &w)
    {
        os << "***** Environment *****" << endl
           << endl;
        os << "Symbols: ";
        for (const string &s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (const GroundedCondition &s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (const GroundedCondition &g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (const Action &g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }

    // add getters
    auto get_initial_conditions() const
    {
        return this->initial_conditions;
    }
    auto get_goal_conditions() const
    {
        return this->goal_conditions;
    }
    auto get_actions() const
    {
        return this->actions;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env *create_env(char *filename)
{
    ifstream input_file(filename);
    Env *env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line.empty())
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str())); // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char *line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = {1, 2};
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

class State {
public:
    USet_gcond currConditions;
    std::vector<GroundedAction> actionsSeq;
    float priority;

    State(const USet_gcond& stateConditions, const std::vector<GroundedAction> actionsTillThisState, float p):
        currConditions(stateConditions),
        actionsSeq(actionsTillThisState),
        priority(p) {}

    State applyGAction(const GroundedAction& gAction) const {
        State newState = *this;
        const USet_gcond &groundedEffs = gAction.getGroundedEffects();
        for (const GroundedCondition& gEff: groundedEffs) {
            if (gEff.get_truth()) {
                newState.currConditions.insert(gEff);
            } else {
                newState.currConditions.erase(gEff); // TODO do I need to construct a new one here?
            }
        }
        newState.actionsSeq.push_back(gAction);
        return newState;
    }

    bool checkPreconditions(const GroundedAction& gAction) const {
        const USet_gcond& currStateGConditions= this->currConditions;
        const USet_gcond& preconds = gAction.getGroundedPreconditions();

        for (auto precond_it = preconds.begin(); precond_it != preconds.end(); ++precond_it) {
            if (currStateGConditions.find(*precond_it) == preconds.end()) {
                return false;
            }
        }
        return true;
    }

    friend ostream&operator<<(ostream &os, const State &s)
    {
        os << "Current State: conditions: ";
        for (const auto &c : s.currConditions)
            os << c << ", ";
        os << endl;

        os << "Actions till here: ";
        for (const GroundedAction &a : s.actionsSeq)
            os << a << endl;
        os << endl;
        return os;
    }
};

// Simple hash combination utility
inline void hash_combine(std::size_t& seed, std::size_t hashValue) {
    seed ^= hashValue + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

// Define equality for State (needed for unordered_set)
struct StateEq {
    bool operator()(const State& lhs, const State& rhs) const {
        return lhs.currConditions == rhs.currConditions &&
               lhs.actionsSeq == rhs.actionsSeq &&
               lhs.priority == rhs.priority;
    }
};

// Define hash function for State
struct StateHasher {
    std::size_t operator()(const State& s) const {
        std::size_t seed = 0;

        // Hash all grounded conditions
        for (const auto& cond : s.currConditions) {
            hash_combine(seed, GroundedConditionHasher{}(cond));
        }

        // Hash all actions in the sequence
        for (const auto& act : s.actionsSeq) {
            hash_combine(seed, GroundedActionHasher{}(act));
        }

        // Optionally include priority
        hash_combine(seed, std::hash<float>{}(s.priority));

        return seed;
    }
};

struct StateComparator {
    const std::vector<State>& los;
    StateComparator(const std::vector<State>& ref) : los(ref) {}

    bool operator()(const size_t& lhs, const size_t& rhs) const {
        return los[lhs].priority > los[rhs].priority;
    }
};

std::vector<State> listOfStates; // TODO Can this be a vector? How should this deal with duplicates? Are duplicates possible?
StateComparator stateComparator(listOfStates);
std::priority_queue<size_t, std::vector<size_t>, StateComparator> OpenList {stateComparator};
std::unordered_set<size_t> ClosedList;
std::unordered_map<size_t, vector<vector<string>>> listOfPermutedSymbols;

bool checkIfGoal(const Env* env, const State& currState) {
    const USet_gcond& currentStateConditions = currState.currConditions;
    for (const GroundedCondition& goalCond : env->get_goal_conditions()) {
        if (currentStateConditions.find(goalCond) == currentStateConditions.end())
            return false;
    }
    return true;
}

/**
 * Returns true for the element with the larger value.
 * In a PQ, smaller values are returned before larger values
 *
 * Expected to order PQ as a minheap ordering where small p values are output first
 */
// struct StateComparator {
//     bool operator()(const int& lidx, const int& ridx) const
//     {
//         float lstate = listOfStates[lidx].priority;
//         float rstate = listOfStates[ridx].priority;
//
//         //TODO Check this sign
//         return lstate > rstate;
//     }
// };
list<GroundedAction> planner(Env* env)
{
    // init vars
    std::list<GroundedAction> plan;

    // get all the symbols in the environment as a vector
    std::vector<string> symbolList;
    const unordered_set<string>& symbol_set = env->get_symbols();
    for (auto it = symbol_set.begin(); it != env->get_symbols().end(); ++it) {
        symbolList.push_back(*it);
    }

    // Initializing the first state as start state and putting it in init
    State startState = State(env->get_initial_conditions(), std::vector<GroundedAction>(), 0); // Passing empty actions
    listOfStates.push_back(startState);
    OpenList.push(0);

    State goalState = State(env->get_goal_conditions(), std::vector<GroundedAction>(), 0);

    // run the planner
    while (!OpenList.empty()) {
        size_t currStateIdx = OpenList.top(); OpenList.pop();
        const State currState = listOfStates[currStateIdx];      //TODO Should this be reference? Stale reference issue?
        if (DEBUG) {
            cout<<currState<<"\n**********\n"<<endl;
        }

        if (checkIfGoal(env, currState)) {
            return vector2list(currState.actionsSeq);
        }

        // For all possible actions in the environment
        for (const Action& currGenericAction : env->get_actions()) {
            //Try to generate all possible combinations of inputs
            bool isMemoized = listOfPermutedSymbols.find(currGenericAction.get_args().size()) == listOfPermutedSymbols.end();
            if (isMemoized) {
                listOfPermutedSymbols[currGenericAction.get_args().size()] = getAllSymbolPermutations(symbolList, currGenericAction.get_args().size());
            }

            const std::vector<vector<string>>& allPossibleActions = listOfPermutedSymbols[currGenericAction.get_args().size()];

            for (const vector<string>& groundingArgs: allPossibleActions) {
                list<string> gArgsList = vector2list(groundingArgs);
                GroundedAction currAction = GroundedAction(currGenericAction, vector2list(groundingArgs));

                // checkPreconditionsForThisAction();
                if (currState.checkPreconditions(currAction)) {
                    // for this valid state, nextState = currState.updateState(lastActionTaken) priority = lastValue + 1
                    State nextState = currState.applyGAction(currAction);
                    nextState.priority = currState.priority + 1;

                    if (DEBUG) {
                        cout << "ADDING" << endl;
                        cout<<currState<<endl;
                        cout<<nextState<<"\n**********\n";
                    }

                    // if (currState) has not been visited before
                    listOfStates.push_back(nextState);
                    OpenList.push(listOfStates.size() - 1);
                }
            }
        }
    }

    // The environment consists a set of ST start states and a set of GL goal states.
    // At any time, some subset of actions a[i] can be performed out of set of all actions A
    // The goal of the plan is to move from ST by using a sequence of actions a[1..t] in A that are allowed at any timestep

    // Given current state s
    // For all actions possible in the environment which ones are possible from s_curr?
    //      Check all the preconditions
    // Of all the actions that are possible, which ones should I take? //TODO heuristic?
    //
    return plan;
}

int main(int argc, char *argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    // char *env_file = static_cast<char *>("example.txt");
    const char *env_file = "example.txt";
    if (argc > 1)
        env_file = argv[1];
    std::string envsDirPath = ENVS_DIR;
    char *filename = new char[envsDirPath.length() + strlen(env_file) + 2];
    strcpy(filename, envsDirPath.c_str());
    strcat(filename, "/");
    strcat(filename, env_file);

    cout << "Environment: " << filename << endl;
    Env *env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (const GroundedAction &gac : actions)
    {
        cout << gac << endl;
    }

    delete env;
    return 0;
}